"""GripperState — RG2 소켓 I/O 단독 소유 · 측정/판별/CSV."""
# 자동 분리: grip_web.py → core/ (내용 동일, 위치만 이동)

import json
import os
import threading
import time
import paths                       # 측정 CSV 위치(저장소 동봉 data/)

import grip_config as cfg
from core import runtime as RT
from core.rg2 import ModbusTCP, CONTROL_GRIP, MAX_WIDTH_MM, width_mm_to_angle_deg


# ── 그리퍼 상태 (소켓 I/O 단독 소유) ────────────────────────────────────────
class GripperState(threading.Thread):
    def __init__(self, host, unit):
        super().__init__(daemon=True)
        self.host, self.unit = host, unit
        self.mb = ModbusTCP(host, 502, unit)
        self.lock = threading.Lock()
        self.connected = False
        self.target_width = 60.0
        self.force_n = 20.0
        self.last_sent = None
        self.regs = [0] * 18
        self.actual_width = 0.0
        self.busy = 0
        self.safety = 0
        self._recover_req = False
        self._measure_req = False
        self._measure_loop = False       # 라벨 저장 후 다음 측정을 자동 시작
        self._measure_wait_label = False # 현재 결과에 라벨을 붙이기 전에는 다음 측정 금지
        self._measure_seq = 0
        self._last_saved_measure_seq = 0
        self._training_wait_label = False
        self._training_label = None
        self.measuring = False
        self.measure_result = None    # {'points':[{f,w}], 'size', 'compression', 'class'}
        self.measure_msg = ''
        self.size_offset = 0.0        # 크기 보정(raw너비 - 실제지름)
        self._last_raw_size = None
        self.saved = []               # 저장된 측정 이력(요약)
        self._csv_mtime = 0.0         # CSV 외부변경 감지용(파일↔화면 동기화)
        # ── 자동작업(flow) 그리퍼 동작 ──
        self._flow_req = None         # 'grab'|'measure'|'release'
        self.flow_busy = False
        self.flow_hold = False        # True=자동사이클 중(메인루프 재전송 중지, flow 직접명령 유지)
        self.flow_seq = 0             # flow 요청 시퀀스(ACK용)
        self.flow_done_seq = 0        # 완료된 시퀀스 — 사이클이 이걸 보고 완료 확인
        self.flow_w5 = None           # 접촉 크기
        self.flow_contact_force = None  # 실제 접촉힘(테니스 5N / 정렬 후 야구 10N)
        self.flow_size = None         # 보정된 공 지름(flow_w5 - size_offset)
        self.flow_comp = None         # 총눌림
        self.flow_creep = None        # 크립(40N 유지 표류) — 보조특징
        self.flow_class = None        # 판정
        self.flow_code = None         # 경로 결정용 표준 SKU 코드
        self.flow_ball_type = None    # tennis | baseball
        self.flow_packable = False
        self.flow_error = None        # flow 스레드 오류를 사이클 스레드에 전달
        self.flow_msg = ''
        self._stop = False
        self.status_msg = '연결 중...'

    def request_flow(self, action):
        self.flow_seq += 1
        self.flow_error = None
        self._flow_req = action
        return self.flow_seq

    def save_measurement(self, label):
        """현재 측정결과를 라벨 붙여 이력+CSV에 저장."""
        mr = self.measure_result
        if not mr or not mr.get('points'):
            return False, '저장할 측정이 없습니다'
        measure_seq = int(mr.get('measure_seq') or 0)
        if measure_seq and measure_seq == self._last_saved_measure_seq:
            return False, '이미 라벨을 저장한 측정입니다'
        btype = cfg.LABEL_TYPE.get(label, mr.get('ball_type') or '')
        row = {
            'time': time.strftime('%H:%M:%S'),
            'label': label, 'ball_type': btype,
            'size': mr.get('size'), 'w40': mr.get('w40'),
            'comp_total': mr.get('compression'), 'creep': mr.get('creep'),
            'contact_f': mr.get('contact_f'), 'class': mr.get('class'),
            'points': mr.get('points', []),
        }
        with self.lock:
            self.saved.append(row)
        try:
            path = paths.MEASURE_CSV
            HEADER = ('time,ball_type,label,size_mm,w@40N,comp_total_5to40,creep_1p5s,'
                      'contact_N,verdict,force_width_points\n')
            # 옛 포맷이면 백업 후 새 포맷으로 시작(열 어긋남 방지) — 크립 열 추가로 v1→v2 전환
            if os.path.exists(path):
                with open(path, encoding='utf-8') as rf:
                    first = rf.readline()
                if first != HEADER:
                    os.rename(path, path + '.v1bak')
            newf = not os.path.exists(path)
            _n = lambda v: '' if v is None else v
            with open(path, 'a', encoding='utf-8') as f:
                if newf:
                    f.write(HEADER)
                pj = ';'.join(f"{p['force']:.0f}:{p['width']:.1f}" for p in mr.get('points', []))
                f.write(f"{row['time']},{btype},{label},{_n(row['size'])},{_n(row['w40'])},"
                        f"{_n(row['comp_total'])},{_n(row['creep'])},{_n(row['contact_f'])},"
                        f"{mr.get('class','')},{pj}\n")
            self._csv_mtime = self._csv_mtime_now()   # 내가 쓴 변경은 재로드 불필요
        except Exception as e:
            with self.lock:
                self._last_saved_measure_seq = measure_seq
                if self._training_wait_label:
                    self._training_label = label
                    self._training_wait_label = False
                if self._measure_loop:
                    self._measure_wait_label = False
                    self._measure_req = True
            return True, f'이력에 저장(파일쓰기 실패: {e})'
        with self.lock:
            self._last_saved_measure_seq = measure_seq
            if self._training_wait_label:
                self._training_label = label
                self._training_wait_label = False
            if self._measure_loop:
                self._measure_wait_label = False
                self._measure_req = True
        return True, f'저장됨 ({label}·{btype or "?"}) — 총 {len(self.saved)}개'

    def reset_training_data(self):
        """기존 라벨 CSV를 삭제하지 않고 시각이 붙은 백업으로 이동한다."""
        path = paths.MEASURE_CSV
        backup = None
        if os.path.exists(path):
            stamp = time.strftime('%Y%m%d-%H%M%S')
            backup = f'{path}.bak-{stamp}'
            os.replace(path, backup)
        with self.lock:
            self.saved = []
            self.measure_result = None
            self._last_saved_measure_seq = 0
            self._training_label = None
            self._training_wait_label = False
            self._measure_wait_label = False
        return backup

    def load_saved(self):
        """시작 시 CSV 이력을 self.saved로 복원 → 재시작해도 측정결과·정확도·그래프 유지."""
        import csv as _csv
        path = paths.MEASURE_CSV
        rows = []
        try:
            with open(path, encoding='utf-8') as f:
                for r in _csv.DictReader(f):
                    def _f(k):
                        try:
                            return float(r.get(k))
                        except (TypeError, ValueError):
                            return None
                    rows.append({
                        'time': r.get('time', ''), 'label': (r.get('label') or '').strip(),
                        'ball_type': r.get('ball_type', ''), 'size': _f('size_mm'),
                        'w40': _f('w@40N'), 'comp_total': _f('comp_total_5to40'),
                        'creep': _f('creep_1p5s'),          # v2 열(구버전 CSV엔 없어 None)
                        'contact_f': _f('contact_N'),       # 측정에 쓴 접촉힘(테니스5/야구10)
                        'class': r.get('verdict', ''),
                    })
        except FileNotFoundError:
            return 0
        with self.lock:
            self.saved = rows[-500:]        # 최근 500개만 메모리에(표/그래프용)
        self._csv_mtime = self._csv_mtime_now()
        return len(self.saved)

    @staticmethod
    def _csv_mtime_now():
        try:
            return os.path.getmtime(
                paths.MEASURE_CSV)
        except OSError:
            return 0.0

    def sync_saved(self):
        """CSV가 밖에서 바뀌었으면(수동 편집·행 삭제·초기화) 메모리 이력을 다시 읽는다.
        자동계산은 파일을 직접 읽는데 화면 표(개수·그래프·정확도)는 메모리를 보므로,
        이 동기화가 없으면 재시작 전까지 둘이 어긋난다."""
        m = self._csv_mtime_now()
        if m and m != getattr(self, '_csv_mtime', 0.0):
            return self.load_saved()
        return None

    def compute_thresholds(self):
        """라벨 CSV를 읽어 클래스별 중앙값 → 경계(중간값) 자동계산.

        평균은 한 번의 오파지/미끄러짐에 크게 끌려가므로 중앙값을 사용한다.
        각 클래스 표본 수를 비슷하게 모아야 경계가 한쪽으로 치우치지 않는다.
        return (computed_dict, meta) 또는 (None, err_msg)."""
        import csv as _csv, statistics as _st
        path = paths.MEASURE_CSV
        groups = {}    # label -> {'size':[], 'comp':[], 'w40':[], 'creep':[]}
        skipped = {}   # 접촉힘이 현재 설정과 달라 제외한 행 수(라벨별)
        misgrip = []   # '미검출'(안잡힘/오파지) 라벨의 크기 표본
        try:
            with open(path, encoding='utf-8') as f:
                for r in _csv.DictReader(f):
                    lb = (r.get('label') or '').strip()
                    try:
                        c = float(r.get('comp_total_5to40')); s = float(r.get('size_mm'))
                    except (TypeError, ValueError):
                        continue
                    try:
                        cr = float(r.get('creep_1p5s'))     # v2 열 — 구버전 행엔 없음
                    except (TypeError, ValueError):
                        cr = None
                    try:
                        w4 = float(r.get('w@40N'))          # ★크기−눌림(5N 접촉오차 소거)
                    except (TypeError, ValueError):
                        w4 = s - c                          # 열 없으면 유도(offset=0 기준)
                    if lb == '미검출':
                        misgrip.append(s); continue
                    if lb not in cfg.LABEL_TYPE:
                        continue
                    # ★접촉힘이 지금 설정과 다른 행은 제외. 야구를 10N으로 바꾼 뒤에도 5N 옛 행이
                    #   섞이면 눌림 척도가 달라 경계가 오염된다(조용히 틀리는 게 최악).
                    try:
                        cN = float(r.get('contact_N'))
                    except (TypeError, ValueError):
                        cN = float(cfg.GRIPPER['contact_force'])   # 구버전 행 = 5N
                    want = (cfg.GRIPPER.get('contact_force_baseball', 10.0)
                            if cfg.LABEL_TYPE[lb] == 'baseball' else cfg.GRIPPER['contact_force'])
                    if abs(cN - float(want)) > 0.51:
                        skipped[lb] = skipped.get(lb, 0) + 1
                        continue
                    g = groups.setdefault(lb, {'size': [], 'comp': [], 'w40': [], 'strain': [], 'creep': []})
                    g['comp'].append(c); g['size'].append(s); g['w40'].append(w4)
                    if s:
                        g['strain'].append(c / s)
                    if cr is not None:
                        g['creep'].append(cr)
        except FileNotFoundError:
            return None, 'CSV 없음 — 먼저 라벨을 저장하세요'

        # ── 두 클래스를 한 특징으로 나눌 때의 (경계, 방향, 분리도) — 후보 특징 중 최우수 선택 ──
        def _vals(labels, ft):
            """라벨 하나 또는 여러 개(리스트)의 해당 특징 표본을 모아서 반환."""
            if isinstance(labels, str):
                labels = (labels,)
            out = []
            for lb in labels:
                out.extend(groups.get(lb, {}).get(ft, []))
            return out

        def best_split(la, lb_, feats=cfg.FEATURES):
            """la·lb_ 를 feats 각 특징으로 나눠 정확도 최고인 특징 반환.
            la/lb_ 는 라벨 하나 또는 라벨 리스트(그룹). 캐스케이드 관문은 뒤쪽 등급이
            전부 그 관문을 통과하므로 반드시 '나머지 전체'와 비교해야 한다.

            경계는 '중앙값 중점'이 아니라 실측값 사이를 전수 스캔해 정확도 최대인 곳으로 잡고,
            동률이면 가장 여유(마진) 있는 위치를 고른다. 중앙값 중점은 두 분포가 가깝게 붙어
            있을 때 경계가 표본 안쪽으로 파고들어 오분류를 만든다(실측 무압/유압에서 확인).
            return (feature, boundary, la_is_low, accuracy) 또는 None(표본부족)."""
            best = None
            for ft in feats:
                A = _vals(la, ft); B = _vals(lb_, ft)
                if not A or not B:
                    continue
                ma, mb = _st.median(A), _st.median(B)
                if ma == mb:
                    continue                              # 정보 없음(예: 크립 전부 0.0)
                a_low = ma < mb                           # la가 경계 아래쪽인가
                vals = sorted(set(A + B))
                cands = ([(vals[i] + vals[i + 1]) / 2 for i in range(len(vals) - 1)]
                         or [vals[0]])
                n = len(A) + len(B); pick_c = None
                for bnd in cands:
                    ok = (sum(1 for v in A if (v <= bnd) == a_low)
                          + sum(1 for v in B if (v > bnd) == a_low))
                    margin = min(abs(v - bnd) for v in vals)   # 경계~최근접 표본 거리
                    if pick_c is None or (ok, margin) > (pick_c[0], pick_c[1]):
                        pick_c = (ok, margin, bnd)
                acc = pick_c[0] / n
                if best is None or acc > best[3]:
                    best = (ft, round(pick_c[2], 2), a_low, acc)
            return best

        out = {'tennis': {}, 'baseball': {}}; notes = []
        _FL = cfg.FEATURE_LABELS
        # ① 무압 vs 유압 경계 — 크기/눌림/크기-눌림/크립 중 잘 나뉘는 특징 자동선택
        # ★1단계 관문은 무압 vs (유압+구멍). 유압만 놓고 학습하면 구멍이 이 관문을
        #   통과해 무압으로 포장되는 최악의 오류가 난다(실측: 구멍 8개 중 6개 유출).
        s1 = best_split('무압', ['유압', '구멍'])
        if s1:
            ft, bnd, nopress_low, acc = s1
            out['tennis']['nopress_by'] = ft
            out['tennis']['nopress_max'] = bnd
            out['tennis']['nopress_low'] = nopress_low
            notes.append(f"무압/유압 경계 → {_FL.get(ft, ft)} {bnd} (분리도 {acc*100:.0f}%)")
        else:
            notes.append('무압·유압 표본 부족 → nopress 경계 유지')
        # ② 유압 vs 구멍 경계 — 구멍은 자유크기가 줄어 w40이 겹치므로 보통 눌림이 유리
        s2 = best_split('유압', '구멍')
        if s2:
            ft, bnd, normal_low, acc = s2
            out['tennis']['normal_by'] = ft
            out['tennis']['normal_max'] = bnd
            out['tennis']['normal_low'] = normal_low
            notes.append(f"유압/구멍 경계 → {_FL.get(ft, ft)} {bnd} (분리도 {acc*100:.0f}%)")
        else:
            notes.append('유압·구멍 표본 부족 → normal 경계 유지')

        # ── 야구 하드 vs 소프트 — 크기/눌림/크기-눌림/크립 중 가장 잘 나뉘는 것 자동선택 ──
        bb_best = best_split('하드', '소프트')
        if bb_best:
            ft, bnd, hard_low, acc = bb_best
            out['baseball']['by'] = ft
            out['baseball'][cfg.BB_KEY[ft]] = bnd
            out['baseball']['hard_low'] = hard_low
            notes.append(f"야구 하드/소프트 → {_FL.get(ft, ft)} {bnd} (분리도 {acc*100:.0f}%)")
        else:
            notes.append('하드·소프트 표본 부족 → 야구 경계 유지')
        tsz = [s for lb in ('무압', '유압', '구멍') for s in groups.get(lb, {}).get('size', [])]
        bsz = [s for lb in ('하드', '소프트') for s in groups.get(lb, {}).get('size', [])]
        if tsz and bsz:
            out['type_split'] = round((_st.median(tsz) + _st.median(bsz)) / 2, 1)
        # 미검출(안잡힘) 경계 = 실제 공 최대크기와 오파지 최소크기의 중간
        if misgrip:
            real = tsz + bsz
            if real:
                out['misgrip_min'] = round((max(real) + min(misgrip)) / 2, 1)
                if min(misgrip) <= max(real):
                    notes.append('⚠️ 미검출 크기가 실제 공과 겹침 — 표본/파지 확인')
            else:
                out['misgrip_min'] = round(min(misgrip) - 2.0, 1)
        if skipped:
            notes.append('⚠️ 접촉힘이 현재 설정과 달라 제외: '
                         + ', '.join(f'{k} {v}개' for k, v in skipped.items())
                         + ' — 해당 공은 다시 측정하세요')
        counts = {lb: len(groups[lb]['comp']) for lb in groups}
        if misgrip:
            counts['미검출'] = len(misgrip)
        return out, {'notes': notes, 'counts': counts}

    def request_measure(self):
        self._measure_req = True

    def request_measure_loop(self, on):
        """on=True: 측정 → 사람 라벨 클릭 → 다음 측정을 무한 반복."""
        with self.lock:
            self._measure_loop = bool(on)
            if on:
                self._measure_wait_label = False
                self._measure_req = True
            else:
                self._measure_req = False
        return self._measure_loop

    def stage_flow_measurement_for_training(self):
        """자동 사이클의 실제 접촉힘/40N 측정값을 사진의 라벨 버튼용 결과로 올린다."""
        with self.lock:
            if self.flow_w5 is None or self.flow_comp is None or self.flow_size is None:
                return None
            self._measure_seq += 1
            seq = self._measure_seq
            w5 = float(self.flow_w5)
            comp = float(self.flow_comp)
            w40 = round(w5 - comp, 1)
            contact_f = float(
                self.flow_contact_force or cfg.GRIPPER['contact_force'])
            self.measure_result = {
                'measure_seq': seq,
                'points': [
                    {'force': contact_f, 'width': w5},
                    {'force': 40.0, 'width': w40},
                ],
                'size': float(self.flow_size),
                'w40': w40,
                'compression': comp,
                'contact_f': contact_f,
                'ball_type': self.flow_ball_type,
                'class': self.flow_class,
            }
            self._training_label = None
            self._training_wait_label = True
            return seq

    def training_label(self, measure_seq):
        with self.lock:
            mr = self.measure_result or {}
            if int(mr.get('measure_seq') or 0) != int(measure_seq or 0):
                return None
            return self._training_label

    def cancel_training_label_wait(self, measure_seq):
        """학습 종료 시 아직 라벨이 없는 현재 측정의 대기 상태만 해제한다."""
        with self.lock:
            mr = self.measure_result or {}
            if int(mr.get('measure_seq') or 0) == int(measure_seq or 0):
                self._training_wait_label = False
                self._training_label = None

    def request_calib(self, real_mm):
        """마지막 측정의 raw 크기와 실제 지름 차이를 보정값으로 저장."""
        if self._last_raw_size is not None:
            self.size_offset = round(self._last_raw_size - float(real_mm), 1)

    def set_target(self, w):
        with self.lock:
            self.target_width = max(0.0, min(MAX_WIDTH_MM, float(w)))
        self.flow_hold = False   # 수동 조작 시 자동사이클 hold 해제(중단 후 복구 가능)
        self.last_sent = None

    def set_force(self, n):
        with self.lock:
            self.force_n = max(0.0, min(40.0, float(n)))

    def request_recover(self):
        self._recover_req = True

    def stop(self):
        self._stop = True

    def snapshot(self):
        self.sync_saved()          # CSV가 밖에서 바뀌었으면 화면용 이력도 즉시 반영
        with self.lock:
            r = self.regs
            detected = (not self.busy) and (self.target_width + 4.0) < self.actual_width
            ang = width_mm_to_angle_deg(max(0.0, min(MAX_WIDTH_MM, self.actual_width)))
            return {
                'connected': self.connected,
                'target': round(self.target_width, 1),
                'actual': round(self.actual_width, 1),
                'angle': round(ang, 1),
                'rel_width': round(r[9] / 10.0, 1),
                'offset': round(r[0] / 10.0, 1),
                'force': round(self.force_n, 1),
                'busy': int(self.busy),
                'safety': int(self.safety),
                'grip': (r[10] >> 1) & 1 if len(r) > 10 else 0,   # gsta(reg10) 비트1 = 파지감지
                'detected': bool(detected),
                'thickness': round(self.actual_width, 1) if detected else None,
                'regs': list(r),
                'status': self.status_msg,
                'measuring': self.measuring,
                'measure_loop': self._measure_loop,
                'measure_wait_label': self._measure_wait_label,
                'training_wait_label': self._training_wait_label,
                'measure_msg': self.measure_msg,
                'measure_result': self.measure_result,
                'flow_size': self.flow_size,
                'flow_comp': self.flow_comp,
                'flow_class': self.flow_class,
                'flow_code': self.flow_code,
                'flow_ball_type': self.flow_ball_type,
                'flow_packable': self.flow_packable,
                'flow_error': self.flow_error,
                'saved': list(self.saved),
                'thresholds': cfg.CLASSIFY,
                'speed': {**{seg: cfg.SPEED.get(seg + '_max') for seg in cfg.SPEED_SEGMENTS},
                          'op_speed': (RT.JOG._speed_opspeed if RT.JOG else cfg.DEFAULT_OPERATION_SPEED)},
                'move_defs': cfg.FLOW_MOVES,      # 이동별 속도 슬라이더 구성(단일 출처)
                'moves': {m['key']: round(cfg.move_vel(m['key'], m.get('profile') or 'free'), 1)
                          for m in cfg.FLOW_MOVES},
            }

    def _do_recover(self):
        try:
            self.mb.close()
        except Exception:
            pass
        try:
            m = ModbusTCP(self.host, 502, 63, timeout=2.0)
            m.connect()
            try:
                m.write_registers(0, [2], 63)
            except Exception:
                pass
            m.close()
        except Exception:
            pass
        self.connected = False
        self.last_sent = None
        time.sleep(2.0)

    def _settle(self, force_n, timeout):
        """설정힘으로 닫으라 명령하고, 너비가 '안정될 때까지'(움직임 멈춤) 대기.
        busy=0 이거나 너비 변화<0.2mm 가 연속 유지되면 정착으로 판단. 접촉 보장용."""
        self.mb.write_registers(0, [int(round(force_n * 10)), 0, CONTROL_GRIP])
        t0 = time.time()
        prev = None; stable = 0
        w = self.actual_width
        while time.time() - t0 < timeout:
            r = self.mb.read_holding(258, 18)
            w = r[17] / 10.0
            with self.lock:
                self.regs = r; self.actual_width = w; self.busy = r[10] & 1
            if prev is not None and abs(w - prev) < 0.2:
                stable += 1
            else:
                stable = 0
            prev = w
            # busy 해제 + 안정 2회 연속이면 정착 완료
            if (r[10] & 1) == 0 and stable >= 2:
                break
            time.sleep(0.15)
        return w

    def _approach_contact(self, force_n, timeout):
        """일정한 힘으로 그리퍼를 '한 번' 닫으며 RG2 공식 gSTA(reg[10]) bit1(grip detected)로
        접촉을 확인한다 — OnRobot 드라이버와 동일한 판독(bit0=busy, bit1=grip).

        공식 절차(명령 → busy 상승 → 정지 시 grip 판독)를 지킨다:
          ① 이전 모션이 끝날 때까지(busy=0) 기다린 뒤에만 닫기 명령을 보낸다.
             움직이는 중에 새 명령을 주면 방향 전환 스톨을 파지로 오인한다(거짓 접촉의 원인).
          ② 명령 후 '이 명령의' busy 상승을 본 뒤에만 grip bit를 신뢰한다(잔류 플래그 차단).
             busy 펄스를 폴링이 놓친 경우 대비로 '폭 2mm 이상 실제 감소'를 보조 조건으로 둔다.
        반환: (접촉너비, 파지여부)."""
        # ① 이전 모션 종료 대기 — idle 상태에서만 새 명령(전환 스톨/잔류 플래그 방지)
        t0 = time.time()
        start_w = float(self.actual_width)
        while time.time() - t0 < 2.0:
            try:
                r0 = self.mb.read_holding(258, 18)
            except Exception:
                break
            start_w = r0[17] / 10.0
            with self.lock:
                self.regs = r0; self.actual_width = start_w; self.busy = r0[10] & 1
            if (r0[10] & 1) == 0:
                break
            time.sleep(0.08)
        self.mb.write_registers(0, [int(round(force_n * 10)), 0, CONTROL_GRIP])
        t0 = time.time()
        w = start_w; grip = False
        busy_seen = False
        min_travel = float(cfg.GRIPPER.get('contact_fresh_travel', 0.5))
        while time.time() - t0 < timeout:
            r = self.mb.read_holding(258, 18)
            w = r[17] / 10.0
            with self.lock:
                self.regs = r; self.actual_width = w; self.busy = r[10] & 1
            moving = bool(r[10] & 1)
            busy_seen = busy_seen or moving
            fresh_motion = busy_seen or (start_w - w) >= min_travel
            # ★공식 판독 시점: grip detected = '이동이 물체에 막혀 멈췄다'이므로
            #   busy가 떨어진(정지) 상태에서만 인정한다. 이동 중(busy=1)에 읽으면
            #   이전 명령의 잔류 플래그/과도상태를 접촉으로 오인한다(허공 40N 슬램의 원인).
            if (not moving) and ((r[10] >> 1) & 1) and fresh_motion:
                grip = True; break
            if (r[10] & 1) == 0 and w < cfg.GRIPPER['min_ball_width'] and time.time() - t0 > 1.0:
                break                          # 물체 없이 완전히 닫힘
            time.sleep(0.15)
        return w, grip

    def _measure_at(self, force_n, wait):
        """설정힘으로 완전히 닫으라 명령하고 'wait초' 충분히 기다린 뒤 최종(정착) 너비를 읽는다.
        고정 대기 → 천천히 닫히는 중을 멈춤으로 오인하지 않음.
        물체 있으면 물체에서 멈춤, 없으면 ~0mm까지 닫힘."""
        self.mb.write_registers(0, [int(round(force_n * 10)), 0, CONTROL_GRIP])
        t0 = time.time()
        w = self.actual_width
        while time.time() - t0 < wait:
            r = self.mb.read_holding(258, 18)
            w = r[17] / 10.0
            with self.lock:
                self.regs = r; self.actual_width = w; self.busy = r[10] & 1
            time.sleep(0.15)
        return w

    def _contact_adaptive(self, timeout=6.0, retry=None, msg=None):
        """접촉 감지 — 공통 진입점(측정도구·블록판별·실사이클이 같은 절차를 쓰게 함).

        ① 5N으로 접근해 파지 감지.
        ② 감지 실패면 완전히 열고 자동 재시도(retry회).
        ③ 감지된 크기가 야구(>= type_split)면 '정렬 사이클'을 돈다:
             40N으로 꽉 잡기(빗나간 공을 강제로 그리퍼 중앙으로 밀어냄)
             → 완전 개방 → 형상 회복 대기 → 10N으로 다시 접근해 측정.
           5N→10N으로 힘만 올리면 이미 빗나가게 물린 상태를 더 조일 뿐이라 소용없다.
        반환: (접촉너비, 파지여부, 사용한접촉힘N)"""
        f5 = float(cfg.GRIPPER['contact_force'])
        fbb = float(cfg.GRIPPER.get('contact_force_baseball', 10.0))
        n_retry = int(cfg.GRIPPER.get('measure_retry', 2)) if retry is None else int(retry)
        say = msg if callable(msg) else (lambda _t: None)

        w, grip = 0.0, False
        for att in range(n_retry + 1):
            if att:
                say(f'접촉 감지 실패 — 다시 잡는 중 ({att}/{n_retry})')
                self.mb.write_registers(0, [200, 1100, CONTROL_GRIP])
                self._wait_open(timeout=2.5)
            say(f'{f5:.0f}N 으로 접촉 감지 중...')
            w, grip = self._approach_contact(f5, timeout=timeout)
            if grip and w >= cfg.GRIPPER['min_ball_width']:
                break
        if not grip or w < cfg.GRIPPER['min_ball_width']:
            return w, False, f5

        # 야구공이면 정렬 사이클: 꽉 잡기 → 개방 → 회복 → 재접근
        if w >= cfg.CLASSIFY['type_split'] and fbb > f5:
            fa = float(cfg.GRIPPER.get('align_force', 40.0))
            say(f'야구공 감지({w:.1f}mm) — {fa:.0f}N 꽉 잡아 중앙 정렬')
            self.mb.write_registers(0, [int(round(fa * 10)), 0, CONTROL_GRIP])
            time.sleep(float(cfg.GRIPPER.get('align_hold', 0.8)))
            say('개방 → 형상 회복 대기')
            self.mb.write_registers(0, [200, 1100, CONTROL_GRIP])
            time.sleep(float(cfg.GRIPPER.get('align_recover', 1.8)))
            say(f'{fbb:.0f}N 으로 재접근(정렬 후 측정)')
            w2, grip2 = 0.0, False
            for att in range(n_retry + 1):
                if att:
                    say(f'{fbb:.0f}N 재접근 실패 — 다시 잡는 중 ({att}/{n_retry})')
                    self.mb.write_registers(0, [200, 1100, CONTROL_GRIP])
                    self._wait_open(timeout=2.5)
                w2, grip2 = self._approach_contact(fbb, timeout=timeout)
                if grip2 and w2 >= cfg.GRIPPER['min_ball_width']:
                    return w2, True, fbb
            # 정렬 과정에서 이미 공을 열어 놓았으므로 정렬 전의 5N 폭은 현재 파지상태가 아니다.
            # 그 값을 성공으로 반환하면 공이 없는 상태에서 40N 판정으로 넘어간다.
            say(f'{fbb:.0f}N 재접근 실패 — 판정 중단')
            return w2, False, fbb
        return w, True, f5

    def _wait_open(self, target_w=100.0, timeout=2.0):
        """실제로 열릴 때까지 대기(재시도 경로에서 사용).
        open/release 플로우는 고정 대기를 쓰고, 이 헬퍼는 접촉 재시도 사이에만 쓴다."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = self.mb.read_holding(258, 18)
            w = r[17] / 10.0
            with self.lock:
                self.regs = r; self.actual_width = w; self.busy = r[10] & 1
            if w >= target_w and not (r[10] & 1):
                return True
            time.sleep(0.08)
        return False

    def _measure_creep(self, w_start, hold_s=1.5, force_n=40.0):
        """설정힘(40N)을 유지한 채 너비 표류(크립)를 측정한다.
        내부 공기가 새거나(구멍) 무압인 공은 지속하중에서 계속 눌려 크립이 크고,
        압이 살아있는 공은 금방 평형에 도달해 크립이 작다 — 눌림량과 독립된 보조특징.
        반환: 시작너비 대비 추가로 눌린 양(mm, +면 더 눌림)."""
        self.mb.write_registers(0, [int(round(force_n * 10)), 0, CONTROL_GRIP])
        t0 = time.time(); w = w_start
        while time.time() - t0 < hold_s:
            r = self.mb.read_holding(258, 18); w = r[17] / 10.0
            with self.lock:
                self.regs = r; self.actual_width = w; self.busy = r[10] & 1
            time.sleep(0.15)
        return round(w_start - w, 2)

    def _grip_settle(self, force_n, timeout, start_w):
        """설정힘으로 닫고 너비가 멈출 때까지(점탄성 정착) 대기 후 정착 너비 반환.
        시작너비보다 실제로 눌린 뒤(0.3mm↓)에만 정착 인정 → 아직 안 눌렸는데 멈춤으로 오인 방지.
        (거의 안 눌리는 단단한 물체는 1.8초 후 인정.)"""
        self.mb.write_registers(0, [int(round(force_n * 10)), 0, CONTROL_GRIP])
        t0 = time.time(); prev = None; stable = 0; w = start_w
        while time.time() - t0 < timeout:
            r = self.mb.read_holding(258, 18); w = r[17] / 10.0
            with self.lock:
                self.regs = r; self.actual_width = w; self.busy = r[10] & 1
            moved = (start_w - w) > 0.3 or (time.time() - t0) > 1.8
            if prev is not None and abs(w - prev) < 0.15 and moved:
                stable += 1
            else:
                stable = 0
            prev = w
            if stable >= 3 and time.time() - t0 > 0.8:
                break
            time.sleep(0.2)
        return round(w, 1)

    def grip_classify(self):
        """블록코딩 '판별' 블록용 — 측정도구(_run_measure)와 동일 절차:
        ① 완전 개방 → ② 5N 접촉감지 → ③ 40N 정착 → ④ classify.
        접촉 안 되면 '⚪ 물체 없음'(if의 '못잡음' 조건이 '없음'으로 매칭). 분류 기준은 측정도구와 같음."""
        try:
            self.flow_hold = True       # 메인루프 재전송 차단(측정도구와 동일 이유)
            self.last_sent = None
            self.mb.write_registers(0, [200, 1100, CONTROL_GRIP])   # 완전 개방(측정도구와 동일)
            time.sleep(1.2)
            w5, grip, _fc = self._contact_adaptive(timeout=6.0,
                                                   msg=lambda t: setattr(self, 'flow_msg', t))
            if not grip:                                            # 접촉 없음 = 물건 없음
                self.mb.write_registers(0, [200, 1100, CONTROL_GRIP])
                with self.lock:
                    self.flow_class = '⚪ 물체 없음'
                    self.flow_code = 'misgrip'
                    self.flow_ball_type = None
                    self.flow_packable = False
                    self.flow_comp = None
                    self.flow_w5 = None
                    self.flow_size = None
                return self.flow_class
            w40 = self._grip_settle(40.0, timeout=4.0, start_w=w5)
            creep = self._measure_creep(w40, hold_s=1.5)         # 크립도 측정(훈련과 동일 기준)
            size = round(w5 - self.size_offset, 1)
            comp = round(w5 - w40, 1)
            verdict, ball_type, packable = cfg.classify(size, comp, creep, w40=w40)  # ball_thresholds.json 반영
            with self.lock:
                self.flow_class = verdict
                self.flow_code = cfg.classification_code(verdict, ball_type)
                self.flow_ball_type = ball_type
                self.flow_packable = bool(packable)
                self.flow_comp = comp
                self.flow_creep = creep
                self.flow_w5 = w5
                self.flow_size = size
            return verdict
        except Exception as e:
            with self.lock:
                self.flow_class = '⚠️ 판별오류'
                self.flow_code = 'unknown'
                self.flow_ball_type = None
                self.flow_packable = False
                self.flow_error = str(e)
            return self.flow_class
        finally:
            self.flow_hold = False      # 판별 끝 → 수동 제어 복귀
            self.last_sent = None

    def _run_measure(self):
        """일정힘(5N)으로 닫으며 접촉을 감지하고, 접촉점부터 1N씩 힘을 올려 강성 측정.
        접촉→40N 눌림량이 크면 물렁(불량), 작으면 단단(정상)."""
        F_CONTACT, F_MAX = 5, 40
        pts = []
        try:
            with self.lock:
                self.measuring = True; self.measure_result = None
            # 메인루프(0.12s마다 슬라이더 목표 재전송)가 측정 명령을 덮어쓰지 못하게 잠금
            self.flow_hold = True
            self.last_sent = None
            # 1) 완전 열기 → 물체 놓을 시간(카운트다운)
            self.mb.write_registers(0, [200, 1100, CONTROL_GRIP])
            for s in (1,):
                self.measure_msg = f'그리퍼 열림 — 물체를 놓으세요 ({s}초 후 측정)'
                time.sleep(1)
            # 2) 접촉 감지(실패 시 자동 재시도 + 야구면 10N 재접근으로 정렬)
            def _say(t):
                self.measure_msg = t
            w_c, grip, f_used = self._contact_adaptive(timeout=6.0, msg=_say)
            if not grip:
                self.mb.write_registers(0, [200, 1100, CONTROL_GRIP]); time.sleep(0.8)
                n_try = int(cfg.GRIPPER.get('measure_retry', 2)) + 1
                with self.lock:
                    self.measure_result = {'points': [], 'size': 0, 'compression': 0,
                                           'contact_f': None, 'class': '⚪ 물체 없음',
                                           'detail': [f'{n_try}회 시도했으나 파지 감지 안 됨 → 사이에 물체 없음',
                                                      '물체를 그리퍼 안쪽 중앙에 놓고 다시 측정하세요']}
                    self.measure_msg = '완료'
                return
            F_CONTACT = f_used                               # 실제 사용한 접촉힘(테니스 5N / 야구 10N)
            self.measure_msg = f'✅ 접촉! {w_c:.1f}mm ({F_CONTACT:.0f}N)'
            time.sleep(0.6)                                  # 접촉 메시지 잠깐 보여줌
            pts.append({'force': float(F_CONTACT), 'width': round(w_c, 1)})   # 접촉점
            # 3) 접촉(5N) → 40N 직행. 중간 단계(10·20·30N)를 두지 않는다.
            #    계단식은 각 단계 정착대기 동안 크립이 comp에 섞여 두 물리량이 뒤엉켰다.
            #    이제 크립을 아래에서 따로 재므로, comp=탄성압축 / creep=점탄성표류로 분리된다.
            self.measure_msg = '40N 파지 — 정착 대기...'
            w_40 = self._grip_settle(40.0, timeout=4.0, start_w=w_c)
            pts.append({'force': 40.0, 'width': w_40})
            # 3b) 크립(점탄성 표류): 40N 유지한 채 1.5초 더 눌림 표류 측정 — 눌림량과 독립된 보조특징
            self.measure_msg = '40N 유지 — 크립(표류) 측정...'
            creep = self._measure_creep(w_40, hold_s=1.5); creep_curve = []
            # 4) 열어서 놓기
            self.measure_msg = '측정 완료, 열기'
            self.mb.write_registers(0, [200, 1100, CONTROL_GRIP]); time.sleep(1.0)
            # 5) 특징 계산 — 접촉(5N)/40N 정착값
            w5 = pts[0]['width']
            w_contact = w5
            self._last_raw_size = w_contact
            size = round(w_contact - self.size_offset, 1)         # 접촉 너비 ≈ 자유크기
            comp_total = round(w5 - w_40, 1)                      # 접촉→40N 총 눌림 = 판별 핵심
            # 판정: 크기→공 종류(테니스/야구), 눌림량→등급 (grip_config.classify 공용 로직)
            verdict, ball_type, packable = cfg.classify(size, comp_total, creep, w40=w_40)
            type_name = {'tennis': '테니스공', 'baseball': '야구공'}.get(ball_type, '미상')
            fc = int(F_CONTACT)
            detail = [
                f'① 크기({fc}N 접촉) = {size}mm → {type_name}'
                + ('  · 야구라 10N 재접근으로 정렬' if fc > 5 else ''),
                f'② 총눌림({fc}→40N) = {comp_total}mm  ← 등급 판별 핵심',
                f'③ 크립(40N 1.5s 표류) = {creep}mm  ← 보조특징',
                f'④ 40N 너비 = {w_40}mm',
                f'⑤ 판정: {verdict}' + ('' if packable else '  (불량/재시도)'),
            ]
            cls = verdict
            with self.lock:
                self.measure_result = {'points': pts, 'size': size, 'compression': comp_total,
                                       'w40': w_40,
                                       'creep': creep, 'creep_curve': creep_curve,
                                       'contact_f': float(F_CONTACT), 'ball_type': ball_type,
                                       'class': cls, 'detail': detail}
                self.measure_msg = '완료'
        except Exception as e:
            self.measure_msg = f'측정 실패: {e}'
            self.connected = False
        finally:
            with self.lock:
                self.measuring = False
            self.flow_hold = False      # 측정 끝 → 수동(슬라이더) 제어 복귀
            self.last_sent = None

    def _run_flow(self, act):
        """자동작업용 그리퍼. grab=저힘 접촉 유지 / measure=40N 정착·판정 / release=열기."""
        self.flow_busy = True
        try:
            if act == 'open':
                # 컨테이너 진입 전에 미리 최대 개방(잡는위치에서 벌리면 쌓인 공을 건드림)
                # flow_hold=True → 진입 이동 내내 메인루프가 안 닫음(계속 열림 유지)
                self.flow_hold = True
                self.flow_msg = '그리퍼 최대 개방(진입 준비)'
                self.mb.write_registers(0, [200, 1100, CONTROL_GRIP]); self.last_sent = None
                self._wait_open()      # 실제 열릴 때까지만 대기(이미 열려 있으면 즉시 통과)
                with self.lock:
                    self.flow_w5 = None
                    self.flow_contact_force = None
                    self.flow_size = None
                    self.flow_comp = None
                    self.flow_class = None
                    self.flow_code = None
                    self.flow_ball_type = None
                    self.flow_packable = False
            elif act == 'grab':
                # 이미 경유지에서 개방됨 → 바로 5N 접촉감지로 닫으며 잡기
                self.flow_hold = True   # 잡은 후에도 유지(메인루프가 다시 안 염)
                self.flow_msg = '공 접촉 감지(잡는 중)'
                w5, grip, contact_f = self._contact_adaptive(
                    timeout=6.0, msg=lambda t: setattr(self, 'flow_msg', t))
                if not grip or w5 < cfg.GRIPPER['min_ball_width']:
                    with self.lock:
                        self.flow_w5 = None
                        self.flow_contact_force = None
                        self.flow_size = None
                        self.flow_comp = None
                        self.flow_class = '⚠️ 공 없음'
                        self.flow_code = 'misgrip'
                        self.flow_ball_type = None
                        self.flow_packable = False
                    self.flow_msg = '⚠️ 공을 못 잡음(접촉 실패)'
                    self.flow_hold = False   # 실패 → 수동제어 복귀(그리퍼 슬라이더 값으로)
                else:
                    with self.lock:
                        self.flow_w5 = round(w5, 1)
                        self.flow_contact_force = float(contact_f)
                    # ★40N 원샷 슬램 제거 — 측정 도구와 동일한 정착을 measure에서 수행해야
                    #   comp가 측정 도구와 일치(슬램은 점탄성 오버슛으로 comp 부풀림 → 사이클 오판).
                    #   지금은 실제 접촉힘(테니스5/야구10N) 유지, measure가 40N으로 올린다.
                    self.flow_msg = f'공 접촉·크기 {self.flow_w5}mm (판정 대기)'
            elif act == 'measure':
                if self.flow_w5 is None:
                    with self.lock:
                        self.flow_class = '⚠️ 공 없음'
                        self.flow_code = 'misgrip'
                        self.flow_ball_type = None
                        self.flow_packable = False
                        self.flow_size = None
                        self.flow_comp = None
                    self.flow_msg = '판별 불가(공 없음)'
                else:
                    # 기준 접촉폭 = grab에서 이번 닫기로 확인한 신선한 너비(flow_w5).
                    # ★40N서 힘 뺐다가 다시 읽으면 점탄성 히스테리시스로 값이 작아져 틀림 →
                    #   처음 접촉값을 기억해서 그대로 기준으로 쓴다(사용자 지적 반영).
                    # w40은 _grip_settle로 '정착(멈출 때까지)' 측정 = 수동 '공측정'과 동일 품질.
                    w5 = self.flow_w5
                    contact_f = float(
                        self.flow_contact_force or cfg.GRIPPER['contact_force'])
                    # ★측정 도구(_run_measure)와 동일한 접촉힘→40N 직행 + 크립(중간단계 없음).
                    #   중간단계는 정착대기 동안 크립이 comp에 섞여 두 물리량을 뒤엉키게 했다.
                    self.flow_msg = f'판별: {contact_f:.0f}N→40N 정착 + 크립'
                    w40 = self._grip_settle(40.0, timeout=4.0, start_w=w5)
                    creep = self._measure_creep(w40, hold_s=1.5)   # 크립(훈련/측정도구와 동일 기준)
                    with self.lock:
                        self.actual_width = w40
                    # 판정 후 이송용 40N 유지(현재 40N 상태 그대로)
                    comp = round(w5 - w40, 1)
                    size = round(w5 - self.size_offset, 1)
                    # 놓침 가드: 40N폭이 공 최소폭보다 작으면 공이 빠진 것(거의 다 닫힘)→'불량' 오판 방지
                    if w40 < cfg.GRIPPER['min_hold_width']:
                        with self.lock:
                            self.flow_size = size
                            self.flow_comp = comp
                            self.flow_class = '⚠️ 놓침(파지 실패)'
                            self.flow_code = 'misgrip'
                            self.flow_ball_type = None
                            self.flow_packable = False
                        self.flow_msg = f'놓침: 40N폭 {w40}mm — 공이 빠짐(분류 안 함)'
                        return
                    verdict, ball_type, packable = cfg.classify(size, comp, creep, w40=w40)
                    code = cfg.classification_code(verdict, ball_type)
                    with self.lock:
                        self.flow_size = size
                        self.flow_comp = comp
                        self.flow_creep = creep
                        self.flow_class = verdict
                        self.flow_code = code
                        self.flow_ball_type = ball_type
                        self.flow_packable = bool(packable)
                    self.flow_msg = (
                        f'판정: {verdict} · 크기 {size}mm · '
                        f'{contact_f:.0f}→40N 눌림 {comp}mm')
                    # ③ 라벨 저장용 measure_result 채움 → 기존 라벨 버튼/CSV로 사이클 결과도 저장 가능
                    with self.lock:
                        self.measure_result = {
                            'points': [{'force': contact_f, 'width': w5},
                                       {'force': 40.0, 'width': w40}],
                            'size': size, 'contact_width': w5, 'w40': w40,
                            'compression': comp, 'ball_type': ball_type,
                            'contact_f': contact_f,
                            'code': code, 'packable': bool(packable),
                            'class': verdict, 'source': 'cycle',
                        }
            elif act == 'release':
                self.flow_msg = '그리퍼 열어 공 놓기'
                self.mb.write_registers(0, [200, 1100, CONTROL_GRIP])
                self._wait_open()          # 실제 개방까지만
                self.flow_w5 = None
                self.flow_contact_force = None
                self.flow_hold = False   # 사이클 종료 → 수동제어(슬라이더) 복귀
                self.target_width = 110.0   # 슬라이더도 열림값으로 동기(재전송 시 안 닫히게)
        except Exception as e:
            with self.lock:
                self.flow_error = str(e)
            self.flow_msg = f'그리퍼 오류: {e}'; self.connected = False
        finally:
            self.flow_busy = False
            self.last_sent = None

    def run(self):
        while not self._stop:
            if self._recover_req:
                self._recover_req = False
                self.status_msg = '🔧 그리퍼 전원 사이클 복구 중...'
                self._do_recover()
            if self._measure_req and self.connected:
                self._measure_req = False
                self._run_measure()
                with self.lock:
                    self._measure_seq += 1
                    if self.measure_result is not None:
                        self.measure_result['measure_seq'] = self._measure_seq
                    self._measure_wait_label = bool(
                        self._measure_loop and self.measure_result
                        and self.measure_result.get('points'))
            if self._flow_req and self.connected:
                act = self._flow_req; seq = self.flow_seq; self._flow_req = None
                self._run_flow(act)
                self.flow_done_seq = seq   # ACK: 사이클이 진행 확인(레이스/미소비 방지)
            if not self.connected:
                try:
                    self.mb.close()
                    self.mb.connect()
                    self.connected = True
                except Exception:
                    self.connected = False
                    self.status_msg = '❌ 그리퍼 미연결 — 랜선/전원/IP 확인'
                    time.sleep(0.3)
                    continue
            try:
                # flow_hold(자동사이클 중)이면 슬라이더 목표폭 재전송 중지 →
                # open으로 연 상태/grab으로 40N 쥔 상태가 그대로 유지됨(중간에 안 닫힘/안 열림)
                if not self.flow_hold:
                    with self.lock:
                        tw, fn = self.target_width, self.force_n
                    # ★중복전송 키에 '힘'도 포함해야 한다. 예전엔 너비만 봐서, 폭이 같고
                    #   파지력만 바꾸면(예: 5N→40N, 둘 다 폭 0) 명령이 아예 안 나갔다.
                    #   → "힘을 안 준다 / 한 번 더 눌러야 먹힌다" 증상의 원인.
                    cmd = (round(tw, 1), round(fn, 1))
                    if cmd != self.last_sent:
                        self.mb.write_registers(0, [int(round(fn * 10)), int(round(tw * 10)), CONTROL_GRIP])
                        self.last_sent = cmd
                r = self.mb.read_holding(258, 18)
                with self.lock:
                    self.regs = r
                    self.actual_width = r[17] / 10.0
                    self.busy = r[10] & 1
                    self.safety = r[16]
                    self.status_msg = f'그리퍼 연결됨 · {self.host}'
            except Exception as e:
                self.connected = False
                self.status_msg = f'⚠ 연결 끊김: {e} (재연결)'
                self.mb.close()
            time.sleep(0.12)
