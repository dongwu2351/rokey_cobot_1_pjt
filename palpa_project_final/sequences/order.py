"""주문 파지 사이클 — 진입·파지·판정·라우팅·이송·복귀."""
# 자동 분리: grip_cycle.py → sequences/ (내용 동일, 위치만 이동)

import time

import grip_config as cfg
from .common import CycleStop, TripError, find_wp, find_order_wp


def run_cycle(ctx, first=True, home_return=True, finish_only=False,
              target_item='tennis_normal', training_mode=False,
              order_remaining=None, resume_at_lift=False, start_from_lid=False):
    """ctx: SimpleNamespace(jog, call, 클라이언트들, 메시지클래스들, joints(), waypoints(), state, motion_active)"""
    jog = ctx.jog

    def msg(m):
        jog.cycle_msg = m

    def chk():
        if jog._cycle_stop:
            raise CycleStop()

    def soft_stop():
        """일반 중단/오류는 감속 정지(DR_STO=2). E-stop을 흉내 내지 않는다."""
        rq = ctx.MoveStop.Request()
        try:
            rq.stop_mode = 2
        except Exception: pass
        return ctx.call(ctx.cstop, rq, cto=2.0)

    def robot_state():
        rr = ctx.call(ctx.cstate, ctx.GetRobotState.Request(), cto=1.0)
        return getattr(rr, 'robot_state', -1) if rr else -1

    def motion_idle():
        r = ctx.call(ctx.cchk, ctx.CheckMotion.Request(), cto=0.8)
        return r is not None and getattr(r, 'success', False) and r.status == 0

    def check_state_or_raise():
        st = robot_state()
        if st in (3, 5, 8, 9, 10):
            raise TripError()
        if st in (6, 15):
            raise RuntimeError(f'복구 금지 로봇 상태({st})')
        return st

    def wait_arrival(target, quick=False, timeout_s=None):
        """완료판정: 목표 near_deg 이내 + 정지 연속 confirm_n회 + check_motion 유휴.
        quick=True: 통과성 경유지 — 확정 1회·유휴 1회·정착 0s로 대기 최소(정밀 정지점은 기본값)."""
        m = cfg.MOTION
        cn = 1 if quick else int(m['confirm_n'])                 # 정지 확정 횟수
        ic = 1 if quick else int(m.get('idle_confirm', 1))       # check_motion 유휴 확인 횟수
        ss = 0.0 if quick else m['settle_s']                     # 정착 대기
        timeout_s = float(timeout_s or m['timeout_s'])
        t0 = time.time(); reached = 0; prev = None; offcnt = 0; missing = 0
        last_st = time.time()
        while time.time() - t0 < timeout_s:
            chk()
            c = ctx.joints()
            if not c:
                missing += 1
                if missing >= 5:
                    raise RuntimeError('관절 토픽이 1초 이상 갱신되지 않음')
                time.sleep(m.get('poll_s', 0.2))
                continue
            missing = 0
            near = c and max(abs(c[i] - target[i]) for i in range(6)) < m['near_deg']
            still = c and prev and max(abs(c[i] - prev[i]) for i in range(6)) < m['still_deg']
            if near and still:
                reached += 1
                if reached >= cn:
                    # check_motion IDLE '연속 N회'까지 확인(잔여모션 중 다음 명령 = 쿵 방지)
                    idle_n = 0
                    while m['use_check_motion'] and idle_n < ic:
                        if motion_idle():
                            idle_n += 1
                        else:
                            idle_n = -99   # 아직 모션 있음 → 완료 아님
                            break
                        time.sleep(0.1)
                    if (not m['use_check_motion']) or idle_n >= ic:
                        st = check_state_or_raise()
                        if st == 1:
                            if ss > 0:
                                time.sleep(ss)
                            return True
                        reached = 0
                    reached = 0        # 아직 모션 진행 중 → 계속 대기
            elif still and prev is not None:
                offcnt += 1
                if offcnt >= 6 and motion_idle():   # 완전정지인데 목표 밖
                    check_state_or_raise()
                    return False                     # 단순 오차 잔류 → 재조준
            else:
                reached = 0; offcnt = 0
            prev = c
            if time.time() - last_st > 2.0:          # ★이동 중 주기적 안전정지 감시(45s 무한대기 방지)
                last_st = time.time()
                check_state_or_raise()
            time.sleep(m.get('poll_s', 0.2))
        raise RuntimeError(f"이동 미완료({int(timeout_s)}s) — {jog.cycle_msg}")

    def goto(target, profile='free', quick=False, move=None):
        """안전정지는 실제 접촉일 수 있으므로 자동 복구·재충돌하지 않고 즉시 중단한다.
        move: '속도' 탭 이동 key(있으면 이 이동만의 속도 사용, 없으면 profile 기본)."""
        try:
            return _goto_once(target, profile, quick, move)
        except TripError as exc:
            msg('🛡 안전정지 감지 — 작업자 확인 전 자동 재이동 금지')
            raise RuntimeError('안전정지 감지 — 원인 확인 후 수동 복구 필요') from exc

    def _goto_once(target, profile='free', quick=False, move=None):
        chk()
        cur = ctx.joints()
        if not cur or len(cur) != 6:
            raise RuntimeError('최신 관절값 수신 안 됨(토픽/드라이버 확인)')
        # ★이동별 속도 1:1: '속도' 탭이 정한 이 이동(move)의 deg/s를 그대로 사용(안전 클램프 없음).
        #   op_speed(전역 %)만 두산 컨트롤러에서 곱해지므로, op=100이면 슬라이더값=실제 deg/s.
        v = max(cfg.SPEED['min_vel'], float(cfg.move_vel(move, profile)))
        delta = [target[i] - cur[i] for i in range(6)]
        if max(abs(d) for d in delta) < 0.2:
            if robot_state() != 1 or not motion_idle():
                raise RuntimeError('목표 근처이지만 로봇이 STANDBY/IDLE 상태가 아님')
            return True
        st = check_state_or_raise()
        if st != 1 or not motion_idle():
            raise RuntimeError(f'이동 전 로봇이 STANDBY/IDLE 상태가 아님(state={st})')
        ctx.motion_active.set()
        try:
            req = ctx.MoveJoint.Request()
            req.pos = [float(x) for x in target]  # 절대목표: timeout 뒤 재전송해도 중복 이동 없음
            req.vel = float(v)
            # 가속은 속도에 비례(구간 속도 올리면 가속도 같이 올라 1:1 체감). 하한만 둔다.
            ar = cfg.SPEED['acc_ratio_carry'] if profile in ('carry', 'extract', 'drop') else cfg.SPEED['acc_ratio']
            a = max(cfg.SPEED.get('cycle_min_acc', 10.0), v * ar)
            req.acc = float(a)
            req.time = 0.0; req.radius = 0.0
            req.mode = 0; req.blend_type = 0; req.sync_type = 1
            jog.last_motion = {
                'kind': 'MoveJ', 'stage': jog.cycle_msg,
                'current': [round(float(x), 3) for x in cur],
                'target': [round(float(x), 3) for x in target],
                'vel': round(float(v), 3), 'acc': round(float(a), 3),
                'profile': profile, 'time': time.strftime('%H:%M:%S'),
                'status': 'sent',
            }
            res = ctx.call(ctx.cmj, req, cto=5.0)
            if res is None or not getattr(res, 'success', False):
                jog.last_motion['status'] = 'rejected'
                raise RuntimeError(f'이동 거부 — 자동복구/재전송 금지 · {jog.cycle_msg}')
            # 도착 확인 + 오차 잔류 시 같은 절대목표를 제한적으로 재조준한다.
            travel_s = max(abs(d) for d in delta) / max(v, 1.0)
            timeout_s = max(float(cfg.MOTION['timeout_s']), min(180.0, travel_s * 12.0 + 15.0))
            for _c in range(int(cfg.MOTION.get('correct_n', 2)) + 1):
                if wait_arrival(target, quick=quick, timeout_s=timeout_s):
                    jog.last_motion['status'] = 'arrived'
                    return True
                cur = ctx.joints()
                if not cur:
                    break
                dd = [target[i] - cur[i] for i in range(6)]
                if max(abs(x) for x in dd) < 0.2:
                    return True
                rq2 = ctx.MoveJoint.Request()
                rq2.pos = [float(x) for x in target]
                rq2.vel = float(min(v, 10.0)); rq2.acc = float(min(a, 6.0))
                rq2.time = 0.0; rq2.radius = 0.0
                rq2.mode = 0; rq2.blend_type = 0; rq2.sync_type = 1
                rr = ctx.call(ctx.cmj, rq2, cto=5.0)
                if rr is None or not getattr(rr, 'success', False):
                    raise RuntimeError('잔여 오차 보정 MoveJ 거부')
            jog.last_motion['status'] = 'arrival_failed'
            raise RuntimeError(f'도착 오차 잔류 — {jog.cycle_msg}')
        finally:
            ctx.motion_active.clear()

    def wait_lead(via, lead=None):
        """경유점에 '충분히 가까워질 때까지' 기다렸다가 True 반환 — 다음 명령 투입 시점.

        고정 시간(sleep)으로 다음 명령을 보내면 이동 거리에 따라 전환 지점이 제각각이 된다.
        긴 이동일수록 훨씬 일찍 꺾여 경유점을 통째로 건너뛴 것처럼 보인다
        (실측: P12→P5 115.6°를 60°/s로 갈 때 0.25s면 13% 지점 = 101° 남기고 전환).
        그래서 '남은 최대 관절거리'로 판단해 어느 구간에서든 일정한 코너를 만든다.
        """
        ld = float(cfg.BLEND.get('lead_deg', 12.0) if lead is None else lead)
        t0 = time.time()
        tmo = float(cfg.BLEND.get('lead_timeout_s', 30.0))
        while time.time() - t0 < tmo:
            chk()
            c = ctx.joints()
            if c and len(c) == 6:
                if max(abs(via[i] - c[i]) for i in range(6)) <= ld:
                    return True
            time.sleep(0.02)
        return False        # 타임아웃 — 그래도 다음 명령을 보내 진행(정지 후 이동일 뿐)

    def blend_chain(steps, radius=None, quick_end=False):
        """async MoveJ를 경유점 근처에서 이어 보내는 기존 이송 체인.

        steps: [(target_posj, move_key), ...] — 마지막이 최종 정지점.
        ★중간 경유점은 반드시 '순수 통과용'이어야 한다. 충돌 회피용으로 티칭된
          경유점(P2/P4 슬롯진입, P13/P15 보충접근, P12→P10 뚜껑안착)을 넣으면
          코너를 자르면서 회피가 무효가 되어 충돌한다.
        dsr_controller2의 async MoveJ는 radius를 전달하지 않으므로 정식 블렌딩은 아니다.
        중간점 도착 검증은 하지 않고 lead 거리에서 다음 명령을 보낸다. 사이클 사이 P5의
        진짜 무정지 통과는 아래 spline_return_via_p5()만 담당한다.
        """
        if not cfg.BLEND.get('enabled', False) or len(steps) < 2:
            for st_ in steps:                           # 블렌딩 꺼짐 → 기존 순차 이동
                goto(st_[0], 'carry', move=st_[1])
            return True
        try:
            chk()
            cur = ctx.joints()
            if not cur or len(cur) != 6:
                raise RuntimeError('최신 관절값 수신 안 됨(토픽/드라이버 확인)')
            # 이미 서 있는 선행 경유점은 건너뛴다(전환 체인이 현재 위치에서 시작할 때).
            while len(steps) > 1 and max(abs(steps[0][0][i] - cur[i]) for i in range(6)) < 0.5:
                steps = steps[1:]
            st = check_state_or_raise()
            if st != 1 or not motion_idle():
                raise RuntimeError(f'블렌딩 시작 전 STANDBY/IDLE 아님(state={st})')
            ctx.motion_active.set()
            try:
                for i, st_ in enumerate(steps):
                    tgt, mk = st_[0], st_[1]
                    lead_i = st_[2] if len(st_) > 2 else None
                    last = (i == len(steps) - 1)
                    v = max(cfg.SPEED['min_vel'], float(cfg.move_vel(mk, 'carry')))
                    a = max(cfg.SPEED.get('cycle_min_acc', 10.0),
                            v * cfg.SPEED['acc_ratio_carry'])
                    req = ctx.MoveJoint.Request()
                    req.pos = [float(x) for x in tgt]
                    req.vel = float(v); req.acc = float(a)
                    req.time = 0.0
                    # dsr_controller2의 amovej(async)는 radius 인자를 받지 않는다.
                    # 거짓 반경값을 기록하지 않고, 중간 전환은 wait_lead 시점의 다음 명령이 담당한다.
                    req.radius = 0.0
                    req.mode = 0                        # 절대각 — 오차 누적 없음
                    req.blend_type = 0; req.sync_type = 1   # 비동기(즉시 반환)라야 블렌딩
                    jog.last_motion = {
                        'kind': 'MoveJ(blend)', 'stage': jog.cycle_msg,
                        'target': [round(float(x), 3) for x in tgt],
                        'vel': round(v, 3), 'acc': round(a, 3),
                        'radius': 0.0, 'profile': mk,
                        'time': time.strftime('%H:%M:%S'), 'status': 'sent',
                    }
                    res = ctx.call(ctx.cmj, req, cto=5.0)
                    if res is None or not getattr(res, 'success', False):
                        jog.last_motion['status'] = 'rejected'
                        raise RuntimeError(f'블렌딩 MoveJ 거부({i + 1}/{len(steps)})')
                    if not last:
                        wait_lead(tgt, lead_i)         # 경유점에 근접했을 때 다음 명령 투입
                final = steps[-1][0]
                travel = max(abs(final[i] - cur[i]) for i in range(6))
                timeout_s = max(float(cfg.MOTION['timeout_s']),
                                min(180.0, travel / 5.0 * 12.0 + 20.0))
                if not wait_arrival(final, quick=quick_end, timeout_s=timeout_s):
                    jog.last_motion['status'] = 'arrival_failed'
                    raise RuntimeError(f'블렌딩 체인 도착 미확인 — {jog.cycle_msg}')
                jog.last_motion['status'] = 'arrived'
                return True
            finally:
                ctx.motion_active.clear()
        except TripError as exc:
            msg('🛡 안전정지 감지(블렌딩) — 작업자 확인 전 자동 재이동 금지')
            raise RuntimeError('안전정지 감지 — 원인 확인 후 수동 복구 필요') from exc

    def spline_return_via_p5(next_lift, next_pick=None):
        """공을 놓은 위치에서 P5와 P2/P4를 모두 무정지 통과해 파지위치까지 간다.

        dsr_controller2는 async MoveJ(amovej) 경로에서 radius를 전달하지 않으므로
        MoveJ를 미리 보내는 것만으로는 진짜 블렌딩이 아니다. MoveSplineJoint로
        [P5, next_lift] 두 점을 컨트롤러에 한 명령으로 넘겨 P5 정지를 없앤다.

        슬롯 하강(next_lift→P20/P3)은 스플라인에 넣지 않는다(과거 실기 문제).
        대신 스플라인이 끝나기 전 next_lift 에 lead_deg_slot(기본 10°)만큼
        남았을 때 하강 MoveJ를 따로 보내 P2/P4 정지도 없앤다 — 하강 자체는
        여전히 독립된 단일 MoveJ라 슬롯 진입 경로가 스플라인에 먹히지 않는다.
        """
        chk()
        cur = ctx.joints()
        if not cur or len(cur) != 6:
            raise RuntimeError('P5 연속복귀 전 최신 관절값 수신 안 됨')
        st = check_state_or_raise()
        if st != 1 or not motion_idle():
            raise RuntimeError(f'P5 연속복귀 전 STANDBY/IDLE 아님(state={st})')

        poses = []
        for target in (common, next_lift):
            p = ctx.Float64MultiArray()
            p.data = [float(x) for x in target]
            poses.append(p)
        # P5→P2/P4 구간 둘 다 free 이송이며 현재 기본값은 30deg/s.
        v = min(float(cfg.move_vel('ord_return')),
                float(cfg.move_vel('ord_to_lift')))
        a = max(cfg.SPEED.get('cycle_min_acc', 10.0),
                v * cfg.SPEED['acc_ratio'])
        req = ctx.MoveSplineJoint.Request()
        req.pos = poses
        req.pos_cnt = len(poses)
        req.vel = [v] * 6
        req.acc = [a] * 6
        req.time = 0.0
        req.mode = 0
        req.sync_type = 1
        jog.last_motion = {
            'kind': 'MoveSJ(return)', 'stage': jog.cycle_msg,
            'target': [round(float(x), 3) for x in next_lift],
            'via': [round(float(x), 3) for x in common],
            'vel': round(v, 3), 'acc': round(a, 3),
            'profile': 'return_via_p5',
            'time': time.strftime('%H:%M:%S'), 'status': 'sent',
        }
        ctx.motion_active.set()
        try:
            res = ctx.call(ctx.cspl, req, cto=5.0)
            if res is None or not getattr(res, 'success', False):
                jog.last_motion['status'] = 'rejected'
                raise RuntimeError('P5 연속복귀 MoveSplineJoint 거부')
            travel = max(abs(next_lift[i] - cur[i]) for i in range(6))
            timeout_s = max(float(cfg.MOTION['timeout_s']),
                            min(180.0, travel / max(v, 1.0) * 12.0 + 20.0))
            if next_pick is None:
                if not wait_arrival(next_lift, quick=True, timeout_s=timeout_s):
                    jog.last_motion['status'] = 'arrival_failed'
                    raise RuntimeError('P5 연속복귀 도착 미확인')
                jog.last_motion['status'] = 'arrived'
                return True
            # ★스플라인이 lift에서 끝나는 것을 확인한 '뒤에만' 하강을 보낸다.
            #   (舊: 10° 남은 시점에 amovej 선행 전송 → 실기 컨트롤러가 스플라인 실행 중
            #    새 모션을 충돌로 판단해 알람 없이 정지(흰불) → '도착 미확인' 배치중단)
            #   quick 검증이라 대기는 ~0.3s — 스플라인 끝의 자연 감속과 겹쳐 체감 정지 없음.
            if not wait_arrival(next_lift, quick=True, timeout_s=timeout_s):
                # 목표 밖 정지 등 오차 — 배치를 중단하지 않고 절대목표 재조준으로 수습
                msg('⑨ 경유지 재조준(스플라인 종료 오차)')
                goto(next_lift, 'free', quick=True, move='ord_to_lift')
            # 하강은 재조준 로직(correct_n)이 있는 goto로 — 일시 오차가 곧장 중단이 되지 않는다
            goto(next_pick, 'approach', move='ord_descend')
            jog.last_motion['status'] = 'arrived'
            return True
        finally:
            ctx.motion_active.clear()

    def grip(act, timeout=12.0):
        """그리퍼 flow 요청 + ACK 대기. 무응답이면 그리퍼 복구(재연결) 후 1회 재시도."""
        for attempt in (0, 1):
            seq = ctx.state.request_flow(act)
            t0 = time.time()
            while time.time() - t0 < timeout:
                chk()
                if ctx.state.flow_done_seq >= seq and not ctx.state.flow_busy:
                    if getattr(ctx.state, 'flow_error', None):
                        raise RuntimeError(f'그리퍼 {act} 실패: {ctx.state.flow_error}')
                    return True
                time.sleep(0.1)
            if attempt == 0:   # ★배치 중 그리퍼 스레드 멈춤(실측: 폭61.9에 target110 미전달) 자가복구
                jog.cycle_msg = f'그리퍼 무응답({act}) → 자동 복구 후 재시도'
                try: ctx.state.request_recover()
                except Exception: pass
                time.sleep(3.0)
        raise RuntimeError(f'그리퍼 응답없음({act}) — 그리퍼 연결/전원 확인')

    def grip_async(act):
        return ctx.state.request_flow(act)

    def grip_wait(seq, timeout=12.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            chk()
            if ctx.state.flow_done_seq >= seq and not ctx.state.flow_busy:
                if getattr(ctx.state, 'flow_error', None):
                    raise RuntimeError(f'그리퍼 open 실패: {ctx.state.flow_error}')
                return True
            time.sleep(0.1)
        raise RuntimeError('그리퍼 응답없음(open) — 그리퍼 연결/전원 확인')

    # ── 주문용 웨이포인트 결정 ───────────────────────────────────────────────
    desired = cfg.normalize_order_target(target_item)
    if desired not in cfg.ORDER_TARGET_LABELS:
        msg(f'⚠️ 지원하지 않는 주문 품목: {target_item}')
        return {'status': 'config_error', 'target': str(target_item)}
    desired_type = cfg.target_ball_type(desired)
    desired_label = cfg.ORDER_TARGET_LABELS.get(desired, desired)
    wps = ctx.waypoints()
    points = {key: find_order_wp(wps, key) for key in cfg.ORDER_WP_NAMES}
    source_pick = points[f'{desired_type}_pick']
    source_lift = points[f'{desired_type}_lift']
    required = ('common', 'pack', 'defect', 'baseball_refill_via',
                'baseball_refill_drop', 'tennis_refill_via', 'tennis_refill_drop')
    missing = [key for key in required if not points[key]]
    if not source_pick:
        missing.append(f'{desired_type}_pick')
    if not source_lift:
        missing.append(f'{desired_type}_lift')
    if missing:
        msg('⚠️ 주문 웨이포인트 부족: ' + ', '.join(missing))
        return {'status': 'config_error', 'target': desired}

    common = points['common']
    if finish_only:   # 주문/배치 사이의 표준 대기 위치는 P5(전체 경유지)
        try:
            goto(common, 'free', quick=True, move='ord_return')   # 통과점 — 최소 검증
            msg('P5 전체 경유지 대기 완료')
            return {'status': 'ready', 'target': desired}
        except Exception as e:
            soft_stop()
            msg(f'⚠️ P5 복귀 실패: {e}')
            return {'status': 'error', 'target': desired, 'error': str(e)}

    try:
        cur = ctx.joints()
        if not cur or len(cur) != 6:
            raise RuntimeError('시작 검사 실패: 최신 관절 토픽 없음')
        if first and not start_from_lid:
            # start_from_lid: 뚜껑을 P8에 막 놓은 직후라 P5가 아닌 게 정상이다.
            near_home = max(abs(cur[i] - cfg.HOME_POSJ[i]) for i in range(6)) <= cfg.START_POSE_TOL_DEG
            near_common = max(abs(cur[i] - common[i]) for i in range(6)) <= cfg.START_POSE_TOL_DEG
            if not (near_home or near_common):
                raise RuntimeError(
                    f'시작 자세가 HOME/P5에서 {cfg.START_POSE_TOL_DEG:.0f}° 이상 벗어남 — '
                    '수동으로 안전 자세 확인 후 시작')
        if robot_state() != 1 or not motion_idle():
            raise RuntimeError('시작 검사 실패: 로봇이 STANDBY/IDLE 상태가 아님')
        tool_snapshot = getattr(ctx, 'tool_snapshot', None)
        if not callable(tool_snapshot):
            raise RuntimeError('시작 검사 실패: 공구/TCP 감시 정보 없음')
        tool, tcp, sample_ts = tool_snapshot()
        if not sample_ts or time.time() - sample_ts > cfg.TOOL_SAMPLE_MAX_AGE_S:
            raise RuntimeError('시작 검사 실패: 공구/TCP 정보가 오래됐거나 없음')
        if tool != cfg.REQUIRED_TOOL or tcp != cfg.REQUIRED_TCP:
            raise RuntimeError(
                f'시작 검사 실패: 공구/TCP 불일치 '
                f'(필요 {cfg.REQUIRED_TOOL}/{cfg.REQUIRED_TCP}, 현재 {tool}/{tcp})')
        if first and not getattr(ctx, 'collision_ready', False):
            # collision_ready면 배치 시작 시 이미 설정됨 → 모드 전환(파란불 대기) 생략
            prepare_collision = getattr(ctx, 'prepare_collision', None)
            if not callable(prepare_collision):
                raise RuntimeError('시작 검사 실패: 충돌감도 준비 함수 없음')
            if not prepare_collision(cfg.COLLISION['fixed']):
                raise RuntimeError(
                    f"시작 검사 실패: 고정 충돌감도({cfg.COLLISION['fixed']}) "
                    'MANUAL 설정/AUTONOMOUS 복귀 실패')
        ensure_auto = getattr(ctx, 'ensure_auto', None)
        if callable(ensure_auto) and not ensure_auto():
            raise RuntimeError('시작 검사 실패: AUTONOMOUS 모드 전환 불가')

        # 첫 사이클은 P5→종류별 lift→pick, 다음 사이클은 return spline의 끝인 lift에서 시작한다.
        oseq = grip_async('open')
        if resume_at_lift:
            # 이전 사이클이 P5·P2/P4 를 모두 무정지 통과해 이미 파지위치에 서 있다.
            # 이동이 없으므로 그리퍼 개방 확인만 하고 바로 잡는다.
            msg(f'②③ 무정지 통과 완료 — 바로 파지 · 목표 {desired_label}')
            grip_wait(oseq)
        else:
            msg('① 그리퍼 개방 확인')
            grip_wait(oseq)      # 슬롯 진입 전 개방 보장(놓기 직후면 즉시 통과)
            entry = []
            if start_from_lid:
                # 뚜껑을 P8에 놓은 직후 → 뚜껑 이탈 경유지(P9)를 체인 맨 앞에 붙여
                # P9 → P5 → P2 → P20 을 한 번의 연속 이동으로 실행한다(중간 정지 0회).
                lid_via = None
                for _want in cfg.LID_WP_NAMES['lid_via']:
                    _cw = ''.join(str(_want).split())
                    for _w in wps:
                        if _cw in ''.join(str(_w.get('name', '')).split()) and _w.get('posj'):
                            lid_via = list(_w['posj']); break
                    if lid_via:
                        break
                if lid_via:
                    entry.append((lid_via, 'lidopen_near', cfg.BLEND.get('lead_deg', 15.0)))
            entry += [(common, 'ord_enter', cfg.BLEND.get('lead_deg_hub', 45.0)),
                      (source_lift, 'ord_to_lift', cfg.BLEND.get('lead_deg_slot', 15.0)),
                      (source_pick, 'ord_descend')]
            msg(f'②③ {"P9·" if start_from_lid else ""}P5·경유지 무정지 통과→잡는 위치 · 목표 {desired_label}')
            blend_chain(entry)

        msg('④ 공 잡기(저힘 접촉감지)')
        grip('grab')
        if ctx.state.flow_w5 is None:
            msg('⚠️ 공 못 잡음 → P5 안전 복귀')
            goto(source_lift, 'approach')
            goto(common, 'free')
            msg(f'🚨 {desired_type} 슬롯에 공이 없습니다')
            return {'status': 'empty', 'target': desired}

        # 테니스 P20/야구 P3에서 바로 크기+강성 판정.
        # 화면문구가 아닌 flow_code로 경로를 결정한다.
        msg('⑤ 판정 중(잡는 위치·접촉힘→40N 정착)')
        grip('measure')
        cls = ctx.state.flow_class or ''
        observed = getattr(ctx.state, 'flow_code', None) or cfg.classification_code(
            cls, getattr(ctx.state, 'flow_ball_type', None))

        # 학습 루프에서는 자동판정으로 분기하지 않는다. 측정값을 대시보드 라벨
        # 버튼에 올리고 사람이 현재 공의 정답을 누를 때까지 공을 잡은 채 기다린다.
        # 전용 종료는 즉시 정지하지 않고 현재 공을 원래 슬롯에 돌려놓은 뒤 끝낸다.
        training_label = None
        training_stopping = False
        if training_mode:
            stage = getattr(ctx.state, 'stage_flow_measurement_for_training', None)
            if not callable(stage):
                raise RuntimeError('학습 측정 연결 함수 없음')
            measure_seq = stage()
            if not measure_seq:
                raise RuntimeError('학습용 측정값 생성 실패')
            msg(f'🧪 {desired_label} 슬롯 측정 완료 · 화면에서 실제 공 라벨 선택 대기')
            while training_label is None:
                if getattr(jog, '_training_stop', False):
                    training_stopping = True
                    cancel = getattr(ctx.state, 'cancel_training_label_wait', None)
                    if callable(cancel):
                        cancel(measure_seq)
                    break
                # 비상/일반 작업중단은 기존처럼 즉시 감속정지한다.
                chk()
                training_label = ctx.state.training_label(measure_seq)
                if training_label is None:
                    time.sleep(0.1)

        if training_mode:
            # 빠른 반복 학습: 측정한 공을 현재 P20/P3에 그대로 다시 내려놓고
            # lift(P2/P4)→P5로만 빠져나온다. 슬롯 보충(P15/P16, P13/P14)은 생략한다.
            label_text = training_label or '종료 요청'
            msg(f'⑥ 측정 위치에 공 다시 놓기 · 라벨 {label_text}')
            grip('release')
            time.sleep(0.3)
            w_now = getattr(ctx.state, 'actual_width', None)
            if w_now is not None and w_now < 90.0:
                grip('release')
                time.sleep(0.5)
                w_now = getattr(ctx.state, 'actual_width', None)
                if w_now is not None and w_now < 90.0:
                    raise RuntimeError(f'개방 실패(폭 {w_now}mm) — 측정 공 놓지 못함')
            msg(f'⑦ {desired_type} 측정 위치→P5 복귀')
            goto(source_lift, 'approach', quick=True)
            goto(common, 'free')
            outcome = 'training_stopped' if training_stopping else 'trained'
            if outcome == 'trained':
                msg(f'🧪 학습 표본 저장 완료 · {training_label} · P5 복귀')
            else:
                msg('🧪 학습 루프 종료 · 현재 공 측정 위치 반납 및 P5 복귀 완료')
            return {
                'status': outcome,
                'target': desired,
                'observed': observed,
                'decision': 'training_source_return',
                'class': cls,
                'label': training_label,
                'measure_seq': measure_seq,
            }

        if observed in ('misgrip', 'unknown'):
            msg('⚠️ 놓침/판정불가 → 현재 슬롯에 놓고 재시도')
            grip('release')
            goto(source_lift, 'approach')
            goto(common, 'free')
            msg(f'⚠️ 판정불가({cls}) — P5 복귀')
            return {'status': 'retry', 'target': desired, 'observed': observed}

        decision = cfg.routing_decision(desired, observed, remaining=order_remaining)
        route_via = None
        if decision == 'pack':
            drop = points['pack']
            dest_name = 'P6 포장 위치'
            outcome = 'packed'
        elif decision == 'defect':
            drop = points['defect']
            dest_name = 'P7 불량 위치'
            outcome = 'defect'
        elif decision == 'replenish_baseball':
            route_via = points['baseball_refill_via']
            drop = points['baseball_refill_drop']
            dest_name = 'P13→P14 야구공 슬롯 보충'
            outcome = 'rerouted'
        else:
            # 같은 종류의 다른 등급 또는 테니스공이 야구 슬롯에서 나온 경우:
            # P15→P16으로 돌려보낸 뒤 다시 검색한다.
            route_via = points['tennis_refill_via']
            drop = points['tennis_refill_drop']
            dest_name = 'P15→P16 테니스공 슬롯 보충'
            outcome = 'rerouted'

        # ⑥⑦ 빼내기+이송을 기존 async MoveJ 체인으로 실행한다.
        msg(f'⑥⑦ 빼내기→{dest_name} 이송(블렌딩) · 판정 {cls} / 목표 {desired_label}')
        chain = [(source_lift, 'ord_extract'),
                 (common, 'ord_carry_hub', cfg.BLEND.get('lead_deg_hub', 20.0))]   # P5는 사방이 트여 크게 돌아도 안전
        if route_via:
            chain.append((route_via, 'ord_carry_via'))
        chain.append((drop, 'ord_place'))
        blend_chain(chain)

        msg(f'⑧ {dest_name}에서 그리퍼 개방')
        grip('release')
        time.sleep(0.3)
        w_now = getattr(ctx.state, 'actual_width', None)
        if w_now is not None and w_now < 90.0:
            grip('release')
            time.sleep(0.5)
            w_now = getattr(ctx.state, 'actual_width', None)
            if w_now is not None and w_now < 90.0:
                raise RuntimeError(f'개방 실패(폭 {w_now}mm) — 공 놓지 못함')

        # 포장 카운트의 커밋 지점은 P6에서 실제 개방이 확인된 직후다.
        # 이후 P5 복귀가 실패해도 이미 상자에 들어간 공을 중복 포장하지 않는다.
        if outcome == 'packed':
            jog.batch_packed += 1
            if order_remaining is not None and observed in order_remaining:
                order_remaining[observed] = max(0, int(order_remaining[observed]) - 1)

        # 보충 위치는 전용 경유지를 역순으로 빠져나온 뒤 P5로 복귀(경유지에서 블렌딩).
        # ⑨ 복귀 — ★사이클 경계를 없앤다.
        # 다음에 집을 공이 이미 정해져 있으면 P5에서 멈추지 않고 그 공의 파지 경유지
        # (P2/P4)까지 한 흐름으로 이어 붙인다: P16 →⊙P15 →⊙P5 → P2
        # 그러면 사이클마다 P5에 서서 개방·검증을 기다리던 시간이 통째로 사라진다.
        nxt_lift = None
        nxt_pick = None
        nxt_target = None
        if cfg.BLEND.get('chain_cycles') and order_remaining:
            nxt_target = next((sku for sku, n in order_remaining.items() if int(n) > 0), None)
            if nxt_target:
                nxt_type = cfg.target_ball_type(nxt_target)
                nxt_lift = points.get(f'{nxt_type}_lift')
                nxt_pick = points.get(f'{nxt_type}_pick')
        if nxt_lift and nxt_pick and cfg.BLEND.get('return_spline', False):
            # 보충 위치(P14/P16)는 P13/P15를 정확히 밟아 빠져나온 뒤 스플라인을 시작한다.
            # 충돌회피용 경유점을 곡선이 잘라 먹지 않게 하는 안전 경계다.
            if route_via:
                msg('⑨ 보충 위치 안전 이탈')
                goto(route_via, 'carry', quick=True, move='ord_carry_via')
            msg(f'⑨ P5·경유지 무정지 통과→다음 파지위치 · 다음 {cfg.ORDER_TARGET_LABELS.get(nxt_target, nxt_target)}')
            spline_return_via_p5(nxt_lift, nxt_pick)
        else:
            chain = []
            if route_via:
                chain.append((route_via, 'ord_carry_via'))
            chain.append((common, 'ord_return',
                          cfg.BLEND.get('lead_deg_hub', 20.0)))
            if nxt_lift and nxt_pick:
                # P5(30°)·P2/P4(10°)를 모두 덮어쓰기 전환으로 통과해 파지위치에서만 정지.
                chain.append((nxt_lift, 'ord_to_lift',
                              cfg.BLEND.get('lead_deg_slot', 10.0)))
                chain.append((nxt_pick, 'ord_descend'))
                msg(f'⑨ P5·경유지 무정지 통과→다음 파지위치 · 다음 {cfg.ORDER_TARGET_LABELS.get(nxt_target, nxt_target)}')
                blend_chain(chain, quick_end=True)
            elif (outcome == 'packed' and cfg.BLEND.get('chain_cycles')
                  and order_remaining is not None
                  and not any(int(n) > 0 for n in order_remaining.values())):
                # ★마지막 공 — P5로 복귀하지 않는다. 이어지는 뚜껑 닫기가 현재 위치에서
                #   [P5 → P9 → P8] 체인을 시작해 한 흐름으로 연결된다(P5 정지 제거).
                #   보충 경유지에 있다면 그 이탈만 안전하게 밟고 넘긴다.
                if route_via:
                    goto(route_via, 'carry', quick=True, move='ord_carry_via')
                nxt_lift = None
                msg('⑨ 마지막 공 — 뚜껑 닫기로 연속 진입')
            else:
                nxt_lift = None
                msg('⑨ P5 복귀')
                blend_chain(chain, quick_end=True)

        if outcome == 'packed':
            msg(f'✅ 포장 완료 · {cls} = 주문품 {desired_label}')
        elif outcome == 'defect':
            msg(f'✅ 불량 분리 완료 · {cls} (포장 카운트 제외)')
        else:
            msg(f'🔄 슬롯 새로고침 완료 · {cls} ≠ {desired_label} — 다시 판정')
        return {'status': outcome, 'target': desired, 'observed': observed,
                'decision': decision, 'class': cls,
                'remaining': dict(order_remaining or {}),
                # 다음 사이클이 P5를 거치지 않고 P2/P4에서 바로 하강할 수 있는지
                'at_pick': bool(nxt_lift), 'next_target': nxt_target,
                'at_drop': (nxt_target is None and outcome == 'packed')}

    except CycleStop:
        soft_stop()
        msg('■ 작업 중단됨')
        return {'status': 'stopped', 'target': desired}
    except Exception as e:
        soft_stop()
        msg(f'⚠️ 작업 중단: {e}')
        return {'status': 'error', 'target': desired, 'error': str(e)}
