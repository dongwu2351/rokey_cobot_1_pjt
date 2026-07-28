"""폐기물 처리 — 폐기통 파지·비움·원위치."""
# 자동 분리: grip_cycle.py → sequences/ (내용 동일, 위치만 이동)

import time

import grip_config as cfg
from .common import CycleStop, _find_named, find_order_wp


def run_waste(ctx):
    """폐기물 처리 — 폐기통을 집어 비우고 제자리에 되돌린다.

    P5 → P25 → P26 파지(0mm·5.5N) → P27 → P28 → P29 → P30(비움)
       → P29 → P27 → P26 개방(110mm·20N) → P25 → P5
    모든 경유점은 blend_lead(기본 10°)로 무정지 통과하고, 파지/개방 지점에서만 정지한다.
    """
    jog = ctx.jog

    def msg(m):
        jog.cycle_msg = m

    def chk():
        if jog._cycle_stop:
            raise CycleStop()

    def robot_state():
        rr = ctx.call(ctx.cstate, ctx.GetRobotState.Request(), cto=1.0)
        return getattr(rr, 'robot_state', -1) if rr else -1

    def motion_idle():
        r = ctx.call(ctx.cchk, ctx.CheckMotion.Request(), cto=0.8)
        return r is not None and getattr(r, 'success', False) and r.status == 0

    def grip_to(width, force, wait):
        ctx.state.set_force(float(force))
        ctx.state.set_target(float(width))
        time.sleep(float(wait))
        return getattr(ctx.state, 'actual_width', None)

    def grip_until_held(width, force, timeout=None):
        """폐기통을 '확실히 물 때까지' 기다린다 — 고정 대기 대신 상태 확인.

        고정 sleep은 파지력이 낮으면 아직 닫히는 중인데 출발해 통을 놓치거나 끌 수 있다.
        RG2 공식 파지감지(gSTA bit1)가 서고 폭이 멈춘 뒤에야 성립으로 본다.
        실패하면 완전 개방 후 재시도하고, 그래도 안 되면 예외로 중단한다(들고 가다 놓치는 사고 방지).
        return: 파지 폭(mm)"""
        G_ = cfg.GRIPPER
        tmo = float(timeout if timeout is not None else G_.get('waste_grip_timeout', 6.0))
        lo = float(G_.get('waste_grip_min_w', 15.0))
        hi = float(G_.get('waste_grip_max_w', 105.0))
        for attempt in range(int(G_.get('waste_grip_retry', 1)) + 1):
            if attempt:
                msg(f'🗑 파지 실패 — 재시도 {attempt}')
                ctx.state.set_force(float(G_.get('waste_open_force', 20.0)))
                ctx.state.set_target(110.0)
                time.sleep(1.4)
            ctx.state.set_force(float(force))
            ctx.state.set_target(float(width))
            t0 = time.time(); prev = None; stable = 0
            while time.time() - t0 < tmo:
                chk()
                w_ = getattr(ctx.state, 'actual_width', None)
                busy = bool(getattr(ctx.state, 'busy', 1))
                regs = getattr(ctx.state, 'regs', None) or []
                held = bool((regs[10] >> 1) & 1) if len(regs) > 10 else False
                if w_ is not None:
                    moved_stop = prev is not None and abs(w_ - prev) < 0.15
                    if moved_stop and not busy:
                        stable += 1
                        # 물린 폭이 정상 범위이고(통 손잡이) 파지 플래그가 서면 성립
                        if stable >= 3 and lo <= w_ <= hi and (held or w_ > lo):
                            return w_
                    else:
                        stable = 0
                    prev = w_
                time.sleep(0.1)
        raise RuntimeError(f'폐기통 파지 실패(폭 {prev}mm) — 이동 중단')

    LEAD = float(cfg.BLEND.get('waste_lead_deg', 10.0))

    def chain(steps):
        """[(target, move_key), ...] — 마지막만 정지, 중간은 LEAD°에서 다음 명령 투입."""
        if not cfg.BLEND.get('enabled', False) or len(steps) < 2:
            for st_ in steps:
                _one(st_[0], st_[1], last=True)
            return True
        chk()
        cur = ctx.joints()
        if not cur or len(cur) != 6:
            raise RuntimeError('폐기물 시퀀스: 관절값 미수신')
        while len(steps) > 1 and max(abs(steps[0][0][i] - cur[i]) for i in range(6)) < 0.5:
            steps = steps[1:]          # 이미 서 있는 선행 경유점은 건너뜀
        st = robot_state()
        if st != 1 or not motion_idle():
            raise RuntimeError(f'폐기물 시퀀스 시작 전 STANDBY/IDLE 아님(state={st})')
        ctx.motion_active.set()
        try:
            for i, st_ in enumerate(steps):
                tgt, mk = st_[0], st_[1]
                lead_i = st_[2] if len(st_) > 2 else None
                last = (i == len(steps) - 1)
                _send(tgt, mk)
                if not last:
                    _wait_lead(tgt, lead_i)
        finally:
            ctx.motion_active.clear()
        return _wait_arrival(steps[-1][0])

    def _send(tgt, mk):
        v = max(cfg.SPEED['min_vel'], float(cfg.move_vel(mk)))
        a = max(cfg.SPEED.get('cycle_min_acc', 10.0), v * cfg.SPEED['acc_ratio'])
        rq = ctx.MoveJoint.Request()
        rq.pos = [float(x) for x in tgt]
        rq.vel = float(v); rq.acc = float(a)
        rq.time = 0.0; rq.radius = 0.0
        rq.mode = 0; rq.blend_type = 0; rq.sync_type = 1
        r = ctx.call(ctx.cmj, rq, cto=5.0)
        if r is None or not getattr(r, 'success', False):
            raise RuntimeError(f'폐기물 이동 거부({mk})')

    def _one(tgt, mk, last=True):
        chk()
        ctx.motion_active.set()
        try:
            _send(tgt, mk)
        finally:
            ctx.motion_active.clear()
        return _wait_arrival(tgt)

    def _wait_lead(via, lead=None):
        ld = float(LEAD if lead is None else lead)
        t0 = time.time()
        while time.time() - t0 < float(cfg.BLEND.get('lead_timeout_s', 30.0)):
            chk()
            c = ctx.joints()
            if c and len(c) == 6 and max(abs(via[i] - c[i]) for i in range(6)) <= ld:
                return True
            time.sleep(0.02)
        return False

    def _wait_arrival(final):
        nd = float(cfg.MOTION['near_deg'])
        t0 = time.time(); prev = None; okn = 0
        while time.time() - t0 < 60.0:
            chk()
            c = ctx.joints()
            if (c and max(abs(c[i] - final[i]) for i in range(6)) < nd
                    and prev and max(abs(c[i] - prev[i]) for i in range(6)) < cfg.MOTION['still_deg']):
                okn += 1
                if okn >= 2 and motion_idle():
                    st = robot_state()
                    if st == 1:
                        return True
                    if st in (3, 5, 8, 9, 10):
                        raise RuntimeError(f'폐기물 시퀀스 중 안전정지(state={st})')
            else:
                okn = 0
            prev = c
            time.sleep(0.15)
        raise RuntimeError('폐기물 시퀀스 도착 미확인')

    wps = ctx.waypoints()
    W = {k: _find_named(wps, names) for k, names in cfg.WASTE_WP_NAMES.items()}
    missing = [k for k, v in W.items() if not v]
    if missing:
        return {'status': 'error', 'error': f'폐기물 좌표 미티칭: {missing}'}
    common = find_order_wp(wps, 'common')
    if not common:
        return {'status': 'error', 'error': '전체 경유지(P5) 미티칭'}

    G = cfg.GRIPPER
    try:
        msg('🗑 폐기물 처리 ① P5→P25→P26 접근')
        # 진입 중 개방을 병행(도착 전까지 충분히 열린다)
        ctx.state.set_force(float(G.get('waste_open_force', 20.0)))
        ctx.state.set_target(110.0)
        chain([(common, 'waste_travel', cfg.BLEND.get('waste_lead_hub', 20.0)),
               (W['waste_via'], 'waste_travel'),
               (W['waste_pick'], 'waste_near')])

        msg(f"🗑 ② 폐기통 파지(0mm·{G.get('waste_grip_force', 5.5):.1f}N) — 물림 확인까지 대기")
        w = grip_until_held(0, G.get('waste_grip_force', 5.5))
        msg(f'🗑 ② 파지 확인 · 폭 {w:.1f}mm')

        HUB = float(cfg.BLEND.get('waste_lead_hub', 20.0))
        msg('🗑 ③ P27→P28→P29→P30 비우기')
        chain([(W['waste_lift'], 'waste_near'),
               (W['waste_pre'], 'waste_carry', HUB),      # P28 코너 20°
               (W['waste_drop'], 'waste_carry'),
               (W['waste_dump'], 'waste_dump')])

        # ④ 복귀 — 정지 없이 한 흐름. 문제는 속도가 아니라 '경로'다.
        #    P29를 벗어나는 첫 부분에 걸리는 구조물이 있으므로 P29 코너만 거의 자르지 않아
        #    (waste_lead_tight) 티칭한 경로를 그대로 따라 빠져나온다.
        #    P28부터는 정상 코너로 부드럽게 이어간다. 속도는 전 구간 그대로 최대.
        msg('🗑 ④ P30→P29→P28→P27→P26 복귀 (P29 바짝 통과)')
        chain([(W['waste_drop'], 'waste_dump', cfg.BLEND.get('waste_lead_tight', 4.0)),
               (W['waste_pre'], 'waste_carry'),
               (W['waste_lift'], 'waste_carry'),
               (W['waste_pick'], 'waste_near')])

        msg(f"🗑 ⑤ 폐기통 내려놓기(110mm·{G.get('waste_open_force', 20.0):.0f}N)")
        grip_to(110, G.get('waste_open_force', 20.0), 1.2)

        msg('🗑 ⑥ P25→P5 복귀')
        chain([(W['waste_via'], 'waste_near'),
               (common, 'waste_travel')])
        msg('🗑 폐기물 처리 완료')
        return {'status': 'ok', 'grip_width': w}

    except CycleStop:
        return {'status': 'stopped'}
    except Exception as e:
        msg(f'⚠️ 폐기물 처리 오류: {e}')
        return {'status': 'error', 'error': str(e)}
