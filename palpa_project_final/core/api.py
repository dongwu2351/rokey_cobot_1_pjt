"""HTTP Handler — 모든 /api 엔드포인트와 정적 페이지 서빙."""
# 자동 분리: grip_web.py → core/ (내용 동일, 위치만 이동)

import json
import os
import paths                       # 데이터 파일 위치(저장소 동봉 data/)
import time
from http.server import BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

import grip_config as cfg
from core import runtime as RT
from core.runtime import (WAYPOINTS, WP_LOCK, PROGRAMS, PROG_LOCK,
                          MOTION_ACTIVE, STATE_NAMES)
from core.rg2 import MAX_WIDTH_MM, CONTROL_GRIP
from core.store import (save_waypoints, load_waypoints,
                        save_programs, load_programs, prog_to_text)
from core.legacy_page import PAGE

# ★palpa_ui.html 은 프로젝트 루트에 있고 이 파일은 core/ 안에 있다.
#   dirname(__file__) 은 core/ 를 가리키므로 반드시 한 단계 올라와야 한다.
#   (안 그러면 파일을 못 찾아 legacy_page.PAGE 옛날 UI 로 조용히 폴백된다)
_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, code, body, ctype='application/json'):
        data = body.encode('utf-8') if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype + '; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path in ('/', '/index.html'):
            # PALPA 디자인 콘솔(palpa_ui.html) 우선 서빙 — 없으면 기존 내장 UI 폴백
            try:
                _p = os.path.join(_ROOT_DIR, 'palpa_ui.html')
                with open(_p, encoding='utf-8') as _f:
                    return self._send(200, _f.read(), 'text/html')
            except Exception as _e:
                # 폴백은 조용하면 안 된다 — 옛날 UI가 떠도 이유를 알 수 없게 된다
                print(f'[ui] palpa_ui.html 서빙 실패 → 내장 폴백 UI 사용: {_e}')
            return self._send(200, PAGE, 'text/html')
        if u.path == '/api/status':
            s = RT.STATE.snapshot()
            rb = RT.ROBOT.snapshot() if RT.ROBOT else {'ros': False}
            # 관절각은 토픽 구독(JointSub)에서 — 실시간 + 컨트롤러 부하 0
            if RT.JOINTS:
                pj, last = RT.JOINTS.snapshot()
                if pj and (time.time() - last) < 1.0:
                    rb['posj'] = pj
                    rb['ros'] = True
                    rb['connected'] = True
                    rb['state_name'] = '연결됨'
            s['robot'] = rb
            with WP_LOCK:
                s['waypoints'] = list(WAYPOINTS)
            _tool_snap = RT.ROBOT.tool_snapshot() if RT.ROBOT else (None, None, 0.0)
            s['tool_watch'] = {
                'now': _tool_snap[:2],
                'sample_age_s': (round(time.time() - _tool_snap[2], 2)
                                 if _tool_snap[2] else None),
                'required': [cfg.REQUIRED_TOOL, cfg.REQUIRED_TCP],
                'ok': (_tool_snap[0] == cfg.REQUIRED_TOOL
                       and _tool_snap[1] == cfg.REQUIRED_TCP
                       and bool(_tool_snap[2])
                       and time.time() - _tool_snap[2] <= cfg.TOOL_SAMPLE_MAX_AGE_S),
                'hist': (list(RT.ROBOT.tool_hist) if RT.ROBOT else []),
            }
            s['cycle'] = {'active': bool(RT.JOG.cycle_active) if RT.JOG else False,
                          'msg': RT.JOG.cycle_msg if RT.JOG else '',
                          'flow_class': RT.STATE.flow_class, 'flow_comp': RT.STATE.flow_comp,
                          'flow_code': RT.STATE.flow_code,
                          'flow_ball_type': RT.STATE.flow_ball_type,
                          'flow_packable': RT.STATE.flow_packable,
                          # 팀 통합(palpa) 측정 3값 노출: 지름=보정 접촉크기, 탄성=총눌림, 무게=물체무게
                          'flow_w5': RT.STATE.flow_w5,
                          'flow_contact_force': RT.STATE.flow_contact_force,
                          'measured_diameter': RT.STATE.flow_size,
                          'measured_elasticity': RT.STATE.flow_comp,
                          'measured_weight': s.get('robot', {}).get('weight_est'),
                          'target_item': (RT.JOG._target_item if RT.JOG else None),
                          'target_label': (cfg.ORDER_TARGET_LABELS.get(RT.JOG._target_item, RT.JOG._target_item)
                                           if RT.JOG else None),
                          'classified_item': RT.STATE.flow_code,
                          'decision': ((RT.JOG.last_cycle_result or {}).get('decision') if RT.JOG else None),
                          'attempts': (RT.JOG._batch_attempts if RT.JOG else 0),
                          'batch_target': (RT.JOG._batch_target if RT.JOG else 1),
                          'batch_packed': (RT.JOG.batch_packed if RT.JOG else 0),
                          'batch_completed': (bool(RT.JOG._batch_completed) if RT.JOG else False),
                          'completion_weight_kg': (RT.JOG._batch_weight_kg if RT.JOG else None),
                          'completion_weight_sd': (RT.JOG._batch_weight_sd if RT.JOG else None),
                          'order_requested': (dict(RT.JOG._order_requested) if RT.JOG else {}),
                          'order_remaining': (dict(RT.JOG._order_remaining) if RT.JOG else {}),
                          'order_sequence': (list(RT.JOG._order_sequence) if RT.JOG else []),
                          'last_motion': (dict(RT.JOG.last_motion) if RT.JOG else {})}
            s['orders'] = [dict(o) for o in (RT.JOG._orders if RT.JOG else [])]
            s['prog'] = {'active': (RT.JOG._prog is not None) if RT.JOG else False,
                         'step': (RT.JOG._prog_i if RT.JOG else 0),
                         'total': (RT.JOG._prog_total if RT.JOG else 0),
                         'msg': (RT.JOG._prog_msg if RT.JOG else '')}
            with PROG_LOCK:
                s['programs'] = sorted(PROGRAMS.keys())
            return self._send(200, json.dumps(s))
        if u.path == '/api/set':
            RT.STATE.set_target(q.get('width', ['60'])[0]); return self._send(200, '{"ok":true}')
        if u.path == '/api/force':
            RT.STATE.set_force(q.get('n', ['20'])[0]); return self._send(200, '{"ok":true}')
        if u.path == '/api/recover':
            RT.STATE.request_recover(); return self._send(200, '{"ok":true}')
        if u.path == '/api/measure':
            RT.STATE.request_measure(); return self._send(200, '{"ok":true}')
        if u.path == '/api/robot_training':
            on = q.get('on', ['1'])[0].lower() in ('1', 'true', 'on')
            if not on:
                if RT.JOG:
                    RT.JOG.request_training_stop()
                return self._send(
                    200,
                    json.dumps({
                        'ok': True,
                        'msg': '학습 종료 요청 · 현재 공을 측정 위치에 놓고 P5에서 종료합니다',
                    }, ensure_ascii=False),
                )
            source = q.get('source', ['tennis'])[0].strip().lower()
            if source not in ('tennis', 'baseball'):
                return self._send(
                    400,
                    json.dumps({'ok': False, 'msg': 'source는 tennis 또는 baseball이어야 합니다'},
                               ensure_ascii=False),
                )
            if (not RT.JOG or RT.JOG.cycle_active or RT.JOG._cycle_req or MOTION_ACTIVE.is_set()
                    or RT.JOG._prog is not None):
                return self._send(
                    200,
                    json.dumps({'ok': False, 'msg': '다른 로봇 작업/이동 종료 후 시작하세요'},
                               ensure_ascii=False),
                )
            if RT.STATE.measuring or RT.STATE.flow_busy or RT.STATE._measure_loop:
                return self._send(
                    200,
                    json.dumps({
                        'ok': False,
                        'msg': '기존 그리퍼 측정/학습 루프를 먼저 종료하세요',
                    }, ensure_ascii=False),
                )
            try:
                vel = float(q.get('vel', [str(cfg.DEFAULT_WORK_VEL)])[0])
            except (TypeError, ValueError):
                return self._send(400, '{"ok":false,"msg":"vel 형식 오류"}')
            backup = None
            if q.get('fresh', ['0'])[0].lower() in ('1', 'true', 'on'):
                backup = RT.STATE.reset_training_data()
            accepted = RT.JOG.request_training(source=source, vel=vel)
            if not accepted:
                return self._send(
                    200,
                    json.dumps({'ok': False, 'msg': '로봇 학습 요청 거부'}, ensure_ascii=False),
                )
            source_name = '테니스공(P20)' if source == 'tennis' else '야구공(P3)'
            msg = (f'{source_name} 빠른 학습 무한루프 시작 · '
                   '측정 후 실제 공 라벨을 누르면 같은 위치에 놓고 P5로 복귀합니다')
            if backup:
                msg += f' · 기존 CSV 백업: {os.path.basename(backup)}'
            return self._send(
                200,
                json.dumps({
                    'ok': True, 'training': True, 'source': source,
                    'vel': RT.JOG._cycle_vel, 'msg': msg,
                }, ensure_ascii=False),
            )
        if u.path == '/api/measure_loop':
            on = q.get('on', ['1'])[0].lower() in ('1', 'true', 'on')
            if on and (RT.STATE.measuring or RT.STATE.flow_busy
                       or (RT.JOG and (RT.JOG.cycle_active or RT.JOG._cycle_req))):
                return self._send(
                    200,
                    json.dumps({'ok': False, 'msg': '측정/자동작업 종료 후 학습 루프를 시작하세요'},
                               ensure_ascii=False),
                )
            backup = None
            if on and q.get('fresh', ['0'])[0].lower() in ('1', 'true', 'on'):
                backup = RT.STATE.reset_training_data()
            RT.STATE.request_measure_loop(on)
            if on:
                msg = '라벨 대기형 측정 무한루프 시작'
                if backup:
                    msg += f' · 기존 CSV 백업: {os.path.basename(backup)}'
            else:
                msg = '측정 무한루프 종료(진행 중 측정은 안전하게 마친 뒤 정지)'
            return self._send(
                200,
                json.dumps({'ok': True, 'loop': on, 'msg': msg}, ensure_ascii=False),
            )
        if u.path == '/api/measure_save':
            label = q.get('label', ['미분류'])[0]
            ok, msg = RT.STATE.save_measurement(label)
            return self._send(200, json.dumps({'ok': ok, 'msg': msg}))
        if u.path == '/api/calc_thresholds':      # 라벨 CSV → 임계값 자동계산 + 적용
            out, meta = RT.STATE.compute_thresholds()
            if out is None:
                return self._send(200, json.dumps({'ok': False, 'msg': meta}))
            full = {
                'type_split': out.get('type_split', cfg.CLASSIFY['type_split']),
                'misgrip_min': out.get('misgrip_min', cfg.CLASSIFY.get('misgrip_min', 90.0)),
                'tennis': {**cfg.CLASSIFY['tennis'], **out.get('tennis', {})},
                'baseball': {**cfg.CLASSIFY['baseball'], **out.get('baseball', {})},
            }
            try:
                cfg.save_thresholds(full); cfg.load_thresholds()
            except Exception as e:
                return self._send(200, json.dumps({'ok': False, 'msg': f'저장 실패: {e}'}))
            cnt = meta['counts']; nt = meta['notes']
            _fn = {'size': '크기', 'comp': '눌림', 'creep': '크립'}
            bb = full['baseball']
            _bkey = {'size': 'hard_size_max', 'comp': 'hard_max', 'creep': 'creep_max'}.get(bb.get('by', 'comp'), 'hard_max')
            op = '≤' if bb.get('hard_low', True) else '>'
            bb_txt = f"{_fn.get(bb.get('by', 'comp'), '눌림')}{op}{bb.get(_bkey)}=하드"
            t = full['tennis']
            t_np = _fn.get(t.get('nopress_by', 'comp'), '눌림')
            t_nm = _fn.get(t.get('normal_by', 'comp'), '눌림')
            summary = (f"테니스 무압:{t_np}{'≤' if t.get('nopress_low', True) else '>'}{t.get('nopress_max')} "
                       f"유압:{t_nm}{'≤' if t.get('normal_low', True) else '>'}{t.get('normal_max')} · "
                       f"야구({bb_txt}) · 종류경계 {full['type_split']}mm · 안잡힘≥{full['misgrip_min']}mm")
            return self._send(200, json.dumps({'ok': True,
                'msg': '✅ 임계값 적용: ' + summary,
                'thresholds': full, 'counts': cnt, 'notes': nt}))
        if u.path == '/api/run_waste':     # 폐기물 처리 시퀀스 수동 실행
            if not RT.JOG:
                return self._send(200, '{"ok":false,"msg":"로봇 미연결"}')
            ok = RT.JOG.request_waste()
            return self._send(200, json.dumps(
                {'ok': ok, 'msg': '🗑 폐기물 처리 시작' if ok else '작업 중에는 실행할 수 없습니다'},
                ensure_ascii=False))
        if u.path == '/api/test_move':     # ★속도 탭 ▶이동: 해당 구간을 실제로 재현해 속도 확인
            try:
                key = q.get('key', [''])[0]
                m = cfg.FLOW_MOVE_BY_KEY.get(key)
                if not m:
                    return self._send(200, '{"ok":false,"msg":"알 수 없는 이동"}')
                if not m.get('test', True) or not m.get('to'):
                    return self._send(200, json.dumps(
                        {'ok': False, 'msg': f"{m['label']}은(는) 순응제어 구간이라 단독 재현하지 않습니다"},
                        ensure_ascii=False))
                if not RT.JOG or RT.JOG.cycle_active or RT.JOG._cycle_req:
                    return self._send(200, '{"ok":false,"msg":"작업 중에는 테스트할 수 없습니다"}')
                with WP_LOCK:
                    wps = [dict(w) for w in WAYPOINTS]
                names = {**cfg.ORDER_WP_NAMES, **cfg.LID_WP_NAMES}

                def find(role_key):
                    """ORDER/LID 이름표에서 해당 역할의 웨이포인트 posj를 찾는다."""
                    if not role_key:
                        return None
                    for cand in names.get(role_key, ()):
                        for w in wps:
                            nm = str(w.get('name', '')).strip()
                            if nm == cand or nm.startswith(cand):
                                return list(w['posj'])
                    return None

                src = find(m.get('from'))
                dst = find(m.get('to'))
                if dst is None:
                    return self._send(200, json.dumps(
                        {'ok': False, 'msg': f"도착 웨이포인트 미티칭: {m.get('to')}"}, ensure_ascii=False))
                vel = float(cfg.move_vel(key, m.get('profile') or 'free'))
                ok = RT.JOG.request_move_test(key, m['label'], src, dst, vel)
                if not ok:
                    return self._send(200, '{"ok":false,"msg":"테스트 요청 거부(다른 동작 중)"}')
                return self._send(200, json.dumps(
                    {'ok': True, 'msg': f"{m['label']} 재현 · {vel:.0f}{'mm/s' if m['kind'] != 'j' else '°/s'}",
                     'vel': vel, 'staged': src is not None}, ensure_ascii=False))
            except Exception as e:
                return self._send(200, json.dumps({'ok': False, 'msg': str(e)}, ensure_ascii=False))
        if u.path == '/api/set_speed':     # ★속도 탭: 이동별 속도 + (구)구간 + 전역배속 1:1 적용/저장
            try:
                changed = {}
                for seg in cfg.SPEED_SEGMENTS:                 # 하위호환: 구간 슬라이더가 오면 처리
                    if seg in q:
                        val = max(1.0, min(100.0, float(q.get(seg, ['0'])[0])))
                        cfg.SPEED[seg + '_max'] = val          # 1:1 — 안전 클램프 없음
                        changed[seg] = val
                for mk in cfg.FLOW_MOVE_KEYS:                  # ★이동별 속도 1:1 (핵심)
                    if mk in q:
                        val = max(1.0, min(100.0, float(q.get(mk, ['0'])[0])))
                        cfg.MOVE_SPEED[mk] = val               # 이 이동만의 deg/s(또는 mm/s)
                        changed[mk] = val
                op = None
                if 'op_speed' in q:
                    op = max(1, min(100, int(float(q.get('op_speed', ['100'])[0]))))
                    if RT.JOG:
                        RT.JOG._speed_opspeed = op                # 다음 주문부터 이 배속
                        RT.JOG.request_opspeed(op)                # 지금 즉시도 반영
                cfg.save_speed_profile(op_speed=op)            # 파일로 지속(재시작 유지)
                cur = {m['key']: round(cfg.move_vel(m['key'], m.get('profile') or 'free'), 1)
                       for m in cfg.FLOW_MOVES}
                return self._send(200, json.dumps({'ok': True, 'changed': changed,
                                                   'op_speed': (RT.JOG._speed_opspeed if RT.JOG else op),
                                                   'moves': cur}))
            except Exception as e:
                return self._send(200, json.dumps({'ok': False, 'msg': str(e)}))
        if u.path == '/api/measure_csv':
            path = paths.MEASURE_CSV
            try:
                with open(path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/csv; charset=utf-8')
                self.send_header('Content-Disposition', 'attachment; filename="ball_measurements_final.csv"')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except Exception:
                self._send(404, '아직 저장된 데이터가 없습니다')
            return
        if u.path == '/api/calib':
            try: RT.STATE.request_calib(float(q.get('real', ['0'])[0]))
            except Exception: pass
            return self._send(200, '{"ok":true}')
        if u.path == '/api/tare':
            if RT.ROBOT: RT.ROBOT.request_reset()
            return self._send(200, '{"ok":true}')
        if u.path == '/api/servo_on':
            if RT.ROBOT: RT.ROBOT.request_servo_on()
            return self._send(200, '{"ok":true}')
        if u.path == '/api/servo_off':
            if RT.ROBOT: RT.ROBOT.request_servo_off()
            return self._send(200, '{"ok":true}')
        if u.path == '/api/robot_recover':
            if RT.JOG and (RT.JOG.cycle_active or RT.JOG._cycle_req or MOTION_ACTIVE.is_set()):
                return self._send(200, '{"ok":false,"msg":"이동 중 복구 명령 금지"}')
            if RT.JOG: RT.JOG.request_freedrive(False)   # 자유이동(순응) 남아있으면 해제 — 안 풀면 복구해도 안 움직임
            if RT.ROBOT: RT.ROBOT.request_recover()
            return self._send(200, '{"ok":true}')
        if u.path == '/api/movej':          # 관절 드래그 이동(절대, 안전상 ±40도 제한)
            if RT.ROBOT and RT.JOG:
                try:
                    j = int(q.get('j', ['0'])[0]); val = float(q.get('val', ['0'])[0])
                    vel = float(q.get('vel', ['30'])[0])
                    cur, sample_ts = RT.JOINTS.snapshot() if RT.JOINTS else (None, 0.0)
                    if (cur and len(cur) == 6 and 0 <= j < 6 and sample_ts
                            and time.time() - sample_ts <= cfg.JOINT_SAMPLE_MAX_AGE_S):
                        target = list(cur); target[j] = val
                        ok = RT.JOG.request_goto(
                            'j', target,
                            max(cfg.SPEED['min_vel'], min(cfg.SPEED['manual_max'], vel)))
                        return self._send(200, json.dumps({'ok': ok}))
                except Exception:
                    pass
            return self._send(200, '{"ok":false,"msg":"최신 관절값/요청 형식 확인"}')
        if u.path == '/api/movel':          # 작업축(BASE) 이동 — 상대(d) 또는 절대(val)
            if RT.JOG:
                try:
                    ax = int(q.get('ax', ['0'])[0])
                    vel = float(q.get('vel', ['30'])[0])
                    delta = [0.0] * 6
                    if 'd' in q:                       # ★상대이동(현재 posx 불필요) — 조그용
                        delta[ax] = float(q.get('d', ['0'])[0])
                    else:                              # 절대(현재 posx 필요 · 서비스 응답 시)
                        val = float(q.get('val', ['0'])[0])
                        cur = RT.ROBOT.snapshot().get('posx') if RT.ROBOT else None
                        if not (cur and len(cur) == 6):
                            return self._send(200, '{"ok":false,"msg":"posx 미수신 — 상대(d)로 보내세요"}')
                        delta[ax] = val - cur[ax]
                    RT.JOG.request_goto('l', delta, max(5.0, min(80.0, vel)))
                except Exception:
                    pass
            return self._send(200, '{"ok":true}')
        if u.path == '/api/motion_stop':
            if RT.JOG: RT.JOG.request_motion_stop()
            return self._send(200, '{"ok":true}')
        if u.path == '/api/home':          # 안전 홈자세로(관절이동, 특이점 없음)
            if RT.JOINTS and RT.JOG:
                try:
                    cur, sample_ts = RT.JOINTS.snapshot()
                    if (cur and len(cur) == 6 and sample_ts
                            and time.time() - sample_ts <= cfg.JOINT_SAMPLE_MAX_AGE_S):
                        ok = RT.JOG.request_goto('j', list(cfg.HOME_POSJ), cfg.SPEED['manual_max'])
                        return self._send(200, json.dumps({'ok': ok}))
                except Exception:
                    pass
            return self._send(200, '{"ok":false,"msg":"최신 관절값 없음"}')
        if u.path == '/api/collision':
            return self._send(
                200,
                json.dumps({
                    'ok': False,
                    'msg': (f"충돌감도는 안전상 {cfg.COLLISION['fixed']}으로 고정되며 "
                            '배치/이동 시작 전에 MANUAL에서 설정됩니다.'),
                }, ensure_ascii=False),
            )
        if u.path == '/api/compliance':   # 자유이동(백드라이브)+자동복귀
            if RT.JOG and (RT.JOG.cycle_active or RT.JOG._cycle_req or MOTION_ACTIVE.is_set()):
                return self._send(200, '{"ok":false,"msg":"이동 중 순응모드 변경 금지"}')
            on = q.get('on', ['0'])[0] in ('1', 'true', 'on')
            if RT.JOG: RT.JOG.request_freedrive(on)
            return self._send(200, '{"ok":true}')
        if u.path == '/api/op_speed':     # 전체 이동속도 %(펜던트 오버라이드식, 실시간)
            if RT.JOG and (RT.JOG.cycle_active or RT.JOG._cycle_req or MOTION_ACTIVE.is_set()):
                return self._send(200, '{"ok":false,"msg":"이동 중 전체속도 변경 금지"}')
            try:
                v = int(q.get('v', [str(cfg.DEFAULT_OPERATION_SPEED)])[0])
                if RT.ROBOT: RT.ROBOT.request_opspeed(v)
                elif RT.JOG: RT.JOG.request_opspeed(v)
            except Exception: pass
            return self._send(200, '{"ok":true}')
        if u.path == '/api/start_work':   # 자동작업(배치: count개 포장까지 반복)
            if RT.JOG and (RT.JOG.cycle_active or RT.JOG._cycle_req):
                return self._send(200, '{"ok":false,"msg":"이미 작업 중"}')
            if RT.STATE and (RT.STATE.measuring or RT.STATE._measure_loop):
                return self._send(
                    200,
                    json.dumps({
                        'ok': False,
                        'msg': '측정 학습 루프를 먼저 종료한 뒤 자동작업을 시작하세요',
                    }, ensure_ascii=False),
                )
            try:
                vel = float(q.get('vel', [str(cfg.DEFAULT_WORK_VEL)])[0])
                count = int(q.get('count', ['1'])[0])
            except (TypeError, ValueError):
                return self._send(400, '{"ok":false,"msg":"vel/count 형식 오류"}')
            targets = None
            targets_raw = q.get('targets', [''])[0].strip()
            if targets_raw:
                targets = {}
                try:
                    for part in targets_raw.removeprefix('multi:').split(','):
                        key, raw_qty = part.replace('=', ':', 1).split(':', 1)
                        sku = cfg.normalize_order_target(key)
                        qty = int(raw_qty)
                        if sku not in cfg.ORDER_TARGET_LABELS or qty < 1:
                            raise ValueError(part)
                        targets[sku] = targets.get(sku, 0) + qty
                    count = sum(targets.values())
                except (TypeError, ValueError):
                    return self._send(
                        400,
                        '{"ok":false,"msg":"targets 형식 오류(예: tennis_normal:1,tennis_nopress:1)"}')
            if not 1 <= count <= 3:
                return self._send(400, '{"ok":false,"msg":"주문 수량은 1~3개여야 합니다"}')
            target_raw = (next(iter(targets)) if targets else
                          q.get('target', q.get('target_item', ['tennis_normal']))[0])
            target = cfg.normalize_order_target(target_raw)
            if target not in cfg.ORDER_TARGET_LABELS:
                return self._send(400, '{"ok":false,"msg":"지원하지 않는 주문 품목"}')
            if RT.JOG:
                accepted = RT.JOG.request_cycle(
                    vel, count=count,
                    once=(q.get('once', ['0'])[0] in ('1', 'true')),
                    target=target,
                    targets=targets,
                )
                if not accepted:
                    return self._send(400, '{"ok":false,"msg":"작업 요청 거부"}')
            body = {'ok': True, 'msg': '작업 시작', 'target': target,
                    'targets': targets or {target: count}, 'count': count}
            return self._send(200, json.dumps(body, ensure_ascii=False))
        if u.path == '/api/stop_work':    # 작업 중단
            if RT.JOG: RT.JOG.request_cycle_stop()
            if RT.JOG: RT.JOG.request_motion_stop()
            return self._send(200, '{"ok":true,"msg":"작업 중단"}')
        # ── 포인트 티칭 ──
        if u.path == '/api/wp_save':      # 현재 관절자세(posj) + task좌표(posx) + 그리퍼 상태 저장
            posj = RT.JOINTS.snapshot()[0] if RT.JOINTS else None
            if not posj or len(posj) != 6:
                return self._send(200, '{"ok":false,"msg":"로봇 자세를 못 읽음"}')
            # ★task 좌표(posx=[X,Y,Z,A,B,C])도 함께 저장 → 나중에 MoveL 사용 가능.
            #   RobotMonitor가 GetCurrentPosx로 실시간 캐싱한 값을 사용(없으면 None).
            posx = RT.ROBOT.snapshot().get('posx') if RT.ROBOT else None
            posx = [round(float(x), 2) for x in posx] if (posx and len(posx) == 6) else None
            gw = round(RT.STATE.actual_width, 1) if RT.STATE else 0.0
            with WP_LOCK:
                name = q.get('name', [''])[0] or f'P{len(WAYPOINTS)+1}'
                WAYPOINTS.append({'name': name, 'posj': [round(x, 2) for x in posj],
                                  'posx': posx,
                                  'gripper': gw, 'note': q.get('note', [''])[0],
                                  'role': q.get('role', [''])[0]})
                n = len(WAYPOINTS)
            save_waypoints()
            xmsg = f' · 좌표 {[round(v,1) for v in posx]}' if posx else ' · 좌표 미수신(posx None)'
            return self._send(200, json.dumps({'ok': True,
                'msg': f'P{n} 저장 (관절 {[round(x,1) for x in posj]}{xmsg})'}))
        if u.path == '/api/wp_update':    # 이름/메모 수정
            try:
                i = int(q.get('i', ['-1'])[0])
                with WP_LOCK:
                    if 0 <= i < len(WAYPOINTS):
                        if 'name' in q: WAYPOINTS[i]['name'] = q['name'][0]
                        if 'note' in q: WAYPOINTS[i]['note'] = q['note'][0]
                        if 'role' in q: WAYPOINTS[i]['role'] = q['role'][0]
                save_waypoints()
            except Exception: pass
            return self._send(200, '{"ok":true}')
        if u.path == '/api/wp_delete':
            try:
                i = int(q.get('i', ['-1'])[0])
                with WP_LOCK:
                    if 0 <= i < len(WAYPOINTS): WAYPOINTS.pop(i)
                save_waypoints()
            except Exception: pass
            return self._send(200, '{"ok":true}')
        if u.path == '/api/wp_clear':
            with WP_LOCK: WAYPOINTS.clear()
            save_waypoints()
            return self._send(200, '{"ok":true}')
        if u.path == '/api/wp_goto':      # 저장된 포인트로 절대 MoveJ
            try:
                i = int(q.get('i', ['-1'])[0])
                cur, sample_ts = RT.JOINTS.snapshot() if RT.JOINTS else (None, 0.0)
                with WP_LOCK:
                    tgt = WAYPOINTS[i]['posj'] if 0 <= i < len(WAYPOINTS) else None
                if (tgt and cur and len(cur) == 6 and sample_ts and RT.JOG
                        and time.time() - sample_ts <= cfg.JOINT_SAMPLE_MAX_AGE_S):
                    ok = RT.JOG.request_goto('j', list(tgt), cfg.SPEED['manual_max'])
                    return self._send(200, json.dumps({'ok': ok}))
            except Exception: pass
            return self._send(200, '{"ok":false,"msg":"웨이포인트/최신 관절값 확인"}')
        if u.path == '/api/prog_run':     # ★블록 프로그램 실행: p=JSON(블록평면리스트), vel
            try:
                blocks = json.loads(q.get('p', ['[]'])[0])
                vel = float(q.get('vel', ['25'])[0])
                if isinstance(blocks, list) and blocks and RT.JOG:
                    ok = RT.JOG.request_prog(blocks, vel)
                    return self._send(200, json.dumps({'ok': ok, 'blocks': len(blocks)}))
            except Exception as e:
                return self._send(200, json.dumps({'ok': False, 'msg': str(e)}))
            return self._send(200, '{"ok":false,"msg":"블록 없음"}')
        if u.path == '/api/prog_run_saved':  # 저장 시퀀스 실행(이름으로)
            name = q.get('name', [''])[0]; vel = float(q.get('vel', ['25'])[0])
            with PROG_LOCK:
                blocks = PROGRAMS.get(name)
            if blocks and RT.JOG:
                ok = RT.JOG.request_prog(blocks, vel)
                return self._send(200, json.dumps({'ok': ok, 'name': name, 'blocks': len(blocks)}))
            return self._send(200, '{"ok":false,"msg":"없는 시퀀스"}')
        if u.path == '/api/prog_save':    # 현재 프로그램을 이름 붙여 저장(함수화)
            try:
                name = (q.get('name', [''])[0] or '').strip()
                blocks = json.loads(q.get('p', ['[]'])[0])
                if not name or not isinstance(blocks, list):
                    return self._send(200, '{"ok":false,"msg":"이름/블록 확인"}')
                with PROG_LOCK:
                    PROGRAMS[name] = blocks
                save_programs()
                try:    # 사람·AI가 읽을 수 있는 텍스트 파일도 갱신
                    with PROG_LOCK:
                        allp = dict(PROGRAMS)
                    txt = '\n\n'.join(prog_to_text(nm, bl) for nm, bl in sorted(allp.items()))
                    with open(paths.data_path('sequences_readable.txt'),
                              'w', encoding='utf-8') as f:
                        f.write(txt)
                except Exception:
                    pass
                return self._send(200, json.dumps({'ok': True, 'name': name, 'blocks': len(blocks)}))
            except Exception as e:
                return self._send(200, json.dumps({'ok': False, 'msg': str(e)}))
        if u.path == '/api/prog_get':     # 저장 시퀀스 불러오기(편집/확인용)
            name = q.get('name', [''])[0]
            with PROG_LOCK:
                blocks = PROGRAMS.get(name)
            if blocks is not None:
                return self._send(200, json.dumps({'ok': True, 'name': name,
                                                   'blocks': blocks, 'text': prog_to_text(name, blocks)}))
            return self._send(200, '{"ok":false,"msg":"없음"}')
        if u.path == '/api/prog_delete':
            name = q.get('name', [''])[0]
            with PROG_LOCK:
                PROGRAMS.pop(name, None)
            save_programs()
            return self._send(200, '{"ok":true}')
        if u.path == '/api/prog_download':   # 전체 시퀀스 읽기용 텍스트 다운로드
            with PROG_LOCK:
                allp = dict(PROGRAMS)
            txt = '# ── 저장된 시퀀스(블록코딩) 전체 ──\n\n' + \
                  '\n\n'.join(prog_to_text(nm, bl) for nm, bl in sorted(allp.items()))
            data = txt.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Disposition', 'attachment; filename="sequences_readable.txt"')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers(); self.wfile.write(data); return
        if u.path == '/api/wp_download':  # 티칭 파일 다운로드(사람+AI 읽기용)
            with WP_LOCK:
                lines = ['# 포인트 티칭 파일 (M0609 + RG2)',
                         '# 형식: 번호 | 이름 | posj[J1..J6](deg) | 그리퍼(mm) | 메모', '']
                for idx, w in enumerate(WAYPOINTS):
                    lines.append(f"{idx+1} | {w['name']} | posj={w['posj']} | "
                                 f"gripper={w['gripper']}mm | {w['note']}")
                lines += ['', '# JSON', json.dumps(WAYPOINTS, ensure_ascii=False)]
            data = ('\n'.join(lines)).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Disposition', 'attachment; filename="waypoints.txt"')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        return self._send(404, '{"error":"not found"}')
