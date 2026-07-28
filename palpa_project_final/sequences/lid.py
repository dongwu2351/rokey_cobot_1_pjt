"""뚜껑 시퀀스 — 열기 / 닫기+순응 안착+통 무게 검증."""
# 자동 분리: grip_cycle.py → sequences/ (내용 동일, 위치만 이동)

import time

import grip_config as cfg
from .common import CycleStop, _find_named, _find_named_wp, find_order_wp


def run_lid(ctx, phase):
    """phase='open': P5→P12→P10 뚜껑 잡기(0mm/5N)→P12→P5→P9→P8 내려놓기(110mm/40N)→P5.
    phase='close': P5→P9→P8 잡기(0mm/10N)→P9→P5→P12→[순응ON]→P10 안착→P11 회전잠금→
                   [순응OFF]→기준Fz→+Z100mm 들어올림→Fz→무게=ΔFz/9.81→내려놓기→개방→P12→P5.
    return {'status':'ok'|'error'|'stopped', 'weight_kg':...}"""
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

    def goto(target, vel=12.0, acc=10.0, timeout_s=45.0, near=None, move=None):
        chk()
        if move:                                  # '속도' 탭 이동 key가 있으면 그 속도 사용
            vel = float(cfg.move_vel(move))
            acc = max(cfg.SPEED.get('cycle_min_acc', 10.0), vel * 0.75)
        nd = float(near or cfg.MOTION['near_deg'])
        cur = ctx.joints()
        if not cur or len(cur) != 6:
            raise RuntimeError('관절값 미수신')
        if max(abs(target[i] - cur[i]) for i in range(6)) < 0.2:
            return True
        ctx.motion_active.set()
        try:
            rq = ctx.MoveJoint.Request()
            rq.pos = [float(x) for x in target]
            rq.vel = float(vel); rq.acc = float(acc)
            rq.time = 0.0; rq.radius = 0.0
            rq.mode = 0; rq.blend_type = 0; rq.sync_type = 1
            res = ctx.call(ctx.cmj, rq, cto=5.0)
            if res is None or not getattr(res, 'success', False):
                raise RuntimeError('뚜껑 시퀀스 이동 거부 — 수동 확인 필요')
            t0 = time.time(); prev = None; okn = 0
            while time.time() - t0 < timeout_s:
                chk()
                c = ctx.joints()
                if (c and max(abs(c[i] - target[i]) for i in range(6)) < nd
                        and prev and max(abs(c[i] - prev[i]) for i in range(6)) < cfg.MOTION['still_deg']):
                    okn += 1
                    if okn >= 2 and motion_idle():
                        st = robot_state()
                        if st == 1:
                            time.sleep(0.1)
                            return True
                        if st in (3, 5, 8, 9, 10):
                            raise RuntimeError(f'뚜껑 이동 중 안전정지(state={st}) — 수동 확인')
                else:
                    okn = 0
                prev = c
                time.sleep(0.15)
            raise RuntimeError('뚜껑 이동 도착 미확인')
        finally:
            ctx.motion_active.clear()

    def _wait_lead(via, lead=None):
        """경유점에 근접할 때까지 대기 — 다음 명령 투입 시점(고정시간 금지)."""
        ld = float(cfg.BLEND.get('lead_deg', 12.0) if lead is None else lead)
        t0 = time.time(); tmo = float(cfg.BLEND.get('lead_timeout_s', 30.0))
        while time.time() - t0 < tmo:
            chk()
            c = ctx.joints()
            if c and len(c) == 6 and max(abs(via[i] - c[i]) for i in range(6)) <= ld:
                return True
            time.sleep(0.02)
        return False

    def blend_chain(steps, radius=None, quick_end=False):
        """뚜껑 이송용 radius 블렌딩(MoveJoint.radius[mm] + ASYNC + 절대각).
        steps: [(target_posj, move_key), ...] — 마지막이 정지점.
        ★순응 구간(P10/P11)과 수직 이탈(P10→P12)에는 쓰지 않는다."""
        if not cfg.BLEND.get('enabled', False) or len(steps) < 2:
            for st_ in steps:
                goto(st_[0], move=st_[1])
            return True
        chk()
        cur = ctx.joints()
        if not cur or len(cur) != 6:
            raise RuntimeError('관절값 미수신')
        # 이미 서 있는 선행 경유점은 건너뛴다(전환 체인이 현재 위치에서 시작할 때).
        while len(steps) > 1 and max(abs(steps[0][0][i] - cur[i]) for i in range(6)) < 0.5:
            steps = steps[1:]
        rad = float(cfg.BLEND['radius'] if radius is None else radius)
        ctx.motion_active.set()
        try:
            for i, st_ in enumerate(steps):
                tgt, mk = st_[0], st_[1]
                lead_i = st_[2] if len(st_) > 2 else None
                last = (i == len(steps) - 1)
                v = float(cfg.move_vel(mk))
                a = max(cfg.SPEED.get('cycle_min_acc', 10.0), v * 0.75)
                rq = ctx.MoveJoint.Request()
                rq.pos = [float(x) for x in tgt]
                rq.vel = v; rq.acc = a
                rq.time = 0.0
                # async amovej 경로에서는 드라이버가 radius를 전달하지 않는다.
                rq.radius = 0.0
                rq.mode = 0; rq.blend_type = 0; rq.sync_type = 1
                res = ctx.call(ctx.cmj, rq, cto=5.0)
                if res is None or not getattr(res, 'success', False):
                    raise RuntimeError(f'뚜껑 블렌딩 이동 거부({i + 1}/{len(steps)})')
                if not last:
                    _wait_lead(tgt, lead_i)   # 경유점 근접 시 다음 명령
        finally:
            ctx.motion_active.clear()
        # 최종점 도착만 확인(중간 경유점은 통과하므로 검증 불가)
        # quick_end: 다음 동작으로 바로 이어지는 통과점 — 확정 1회로 대기 최소화
        final = steps[-1][0]
        nd = float(cfg.MOTION['near_deg'])
        need = 1 if quick_end else 2
        t0 = time.time(); prev = None; okn = 0
        while time.time() - t0 < 60.0:
            chk()
            c = ctx.joints()
            if (c and max(abs(c[i] - final[i]) for i in range(6)) < nd
                    and prev and max(abs(c[i] - prev[i]) for i in range(6)) < cfg.MOTION['still_deg']):
                okn += 1
                if okn >= need and motion_idle():
                    st = robot_state()
                    if st == 1:
                        time.sleep(0.1); return True
                    if st in (3, 5, 8, 9, 10):
                        raise RuntimeError(f'뚜껑 블렌딩 중 안전정지(state={st})')
            else:
                okn = 0
            prev = c
            time.sleep(0.15)
        raise RuntimeError('뚜껑 블렌딩 체인 도착 미확인')

    def grip_set(width, force):
        """그리퍼 목표만 걸고 즉시 반환 — 이동과 병행시켜 서 있는 시간을 없앤다.
        도착해서 실제로 잡기 전까지 시간이 충분하면 별도 대기가 필요 없다."""
        ctx.state.set_force(float(force))
        ctx.state.set_target(float(width))

    def grip_to(width, force, wait=1.8):
        """그리퍼 직접 설정(폭mm/힘N) 후 정착 대기 → 실제폭 반환."""
        grip_set(width, force)
        time.sleep(float(wait))
        return getattr(ctx.state, 'actual_width', None)

    def comp_on():
        rq = ctx.TaskComplianceCtrl.Request()
        # 나사 체결용: X/Y·RZ는 단단(위치 유지+회전 토크 전달), Z·RX/RY는 부드럽게(안착·정렬 흡수)
        rq.stx = [2000.0, 2000.0, 500.0, 150.0, 150.0, 300.0]
        rq.ref = 0; rq.time = 0.5
        r = ctx.call(ctx.ccomp_on, rq, cto=2.0)
        if r is None or not getattr(r, 'success', False):
            raise RuntimeError('순응제어 ON 실패')
        time.sleep(0.6)

    def comp_off():
        ctx.call(ctx.ccomp_off, ctx.ReleaseComplianceCtrl.Request(), cto=2.0)
        time.sleep(0.4)

    def movel_z(dz_mm, vmm=20.0):
        """BASE +Z 상대 직선이동(툴이 아래를 봐 툴-Z와 동일 방향). 완료 후 상태 확인."""
        chk()
        ctx.motion_active.set()
        try:
            rq = ctx.MoveLine.Request()
            rq.pos = [0.0, 0.0, float(dz_mm), 0.0, 0.0, 0.0]
            rq.vel = [float(vmm), 10.0]; rq.acc = [float(vmm) * 2.0, 20.0]
            rq.time = 0.0; rq.radius = 0.0; rq.ref = 0
            rq.mode = 1; rq.blend_type = 0; rq.sync_type = 0
            res = ctx.call(ctx.cml, rq, cto=30.0)
            if res is None or not getattr(res, 'success', False):
                raise RuntimeError('통 들어올리기/내려놓기(MoveL) 거부')
        finally:
            ctx.motion_active.clear()
        st = robot_state()
        if st in (3, 5, 8, 9, 10):
            raise RuntimeError(f'직선 이동 중 안전정지(state={st})')

    def movel_abs(posx, vmm=12.0, vdeg=15.0, cto=30.0):
        """절대 posx로 직선(태스크) 이동 — ★순응제어 중 movej는 거부되므로 순응 구간은 이것만 사용."""
        chk()
        ctx.motion_active.set()
        try:
            rq = ctx.MoveLine.Request()
            rq.pos = [float(x) for x in posx]
            rq.vel = [float(vmm), float(vdeg)]; rq.acc = [float(vmm) * 2.0, float(vdeg) * 2.0]
            rq.time = 0.0; rq.radius = 0.0; rq.ref = 0
            rq.mode = 0; rq.blend_type = 0; rq.sync_type = 0
            res = ctx.call(ctx.cml, rq, cto=cto)
            if res is None or not getattr(res, 'success', False):
                raise RuntimeError('순응 구간 직선이동(MoveL) 거부')
        finally:
            ctx.motion_active.clear()
        st = robot_state()
        if st in (3, 5, 8, 9, 10):
            raise RuntimeError(f'순응 구간 이동 중 안전정지(state={st})')

    def _read_fz():
        """Fz(BASE) 1회 읽기 — GetToolForce 직접 호출.
        모니터 스냅샷은 사이클 중 폴링이 멈춰 동결되므로(ΔFz=0 버그) 서비스를 쓴다."""
        if getattr(ctx, 'ctf', None) is not None and getattr(ctx, 'GetToolForce', None) is not None:
            rq = ctx.GetToolForce.Request(); rq.ref = 0      # DR_BASE: 수직축 힘 → 무게 직결
            r = ctx.call(ctx.ctf, rq, cto=1.0)
            tf = getattr(r, 'tool_force', None) if r is not None else None
            tf = list(tf) if tf is not None else []           # ★numpy 진리값 평가 금지
            if len(tf) >= 3:
                return float(tf[2])
            return None
        snap = ctx.robot() if callable(getattr(ctx, 'robot', None)) else {}
        tf = (snap or {}).get('tool_force')
        return float(tf[2]) if tf and len(tf) >= 3 else None

    def _trimmed_mean(vals, trim):
        """상·하위 trim 비율을 잘라낸 평균 — 진동으로 튄 이상치를 제거한다."""
        v = sorted(x for x in vals if x is not None)
        if not v:
            return None, None
        k = int(len(v) * float(trim))
        core = v[k:len(v) - k] if len(v) - 2 * k >= 3 else v
        m = sum(core) / len(core)
        var = sum((x - m) ** 2 for x in core) / len(core)
        return m, var ** 0.5

    def wait_fz_settle():
        """Fz가 실제로 잠잠해질 때까지 기다린다(적응형).

        고정 대기는 통이 뚜껑에 매달려 흔들리는 진자 운동이 남아 있어도 그냥 측정해버려
        측정마다 값이 튀었다. 최근 N개 표본의 표준편차가 임계 이하가 되면 '안정'으로 본다.
        return: 안정까지 걸린 시간(s), 상한 초과 시 None."""
        g = cfg.GRIPPER
        dt = float(g.get('fz_sample_s', 0.1))
        need = int(g.get('fz_settle_n', 8))
        lim = float(g.get('fz_settle_std', 0.35))
        tmax = float(g.get('fz_settle_max_s', 6.0))
        t0 = time.time(); buf = []
        while time.time() - t0 < tmax:
            chk()
            v = _read_fz()
            if v is not None:
                buf.append(v)
                if len(buf) > need:
                    buf.pop(0)
                if len(buf) == need:
                    m = sum(buf) / need
                    sd = (sum((x - m) ** 2 for x in buf) / need) ** 0.5
                    if sd <= lim:
                        return time.time() - t0
            time.sleep(dt)
        return None

    def sample_fz(sec=None):
        """안정 구간의 Fz 절사평균 → (값, 표준편차). 표본은 fz_sample_s 간격으로 촘촘히."""
        g = cfg.GRIPPER
        dt = float(g.get('fz_sample_s', 0.1))
        win = float(sec if sec is not None else g.get('fz_window_s', 1.5))
        t0 = time.time(); vals = []
        while time.time() - t0 < win:
            chk()
            v = _read_fz()
            if v is not None:
                vals.append(v)
            time.sleep(dt)
        return _trimmed_mean(vals, g.get('fz_trim', 0.2))

    wps = ctx.waypoints()
    P = {k: _find_named(wps, names) for k, names in cfg.LID_WP_NAMES.items()}
    missing = [k for k, v in P.items() if not v]
    if missing:
        return {'status': 'error', 'error': f'뚜껑 좌표 미티칭: {missing}'}
    common = find_order_wp(wps, 'common')
    if not common:
        return {'status': 'error', 'error': '전체 경유지(P5) 미티칭'}

    try:
        if phase == 'open':
            msg('🧢 뚜껑 열기 ① P5→P12→P10 접근')
            grip_set(110, 20)          # 개방은 이동과 병행(P10 도착 전까지 충분히 열림)
            goto(common, move='lidopen_travel')
            goto(P['lid_mount_pre'], move='lidopen_travel')
            goto(P['lid_mount'], move='lidopen_near')   # 18mm 하강(뚜껑 위)
            msg('🧢 ② 뚜껑 잡기(폭0·5N)')
            w = grip_to(0, 5, 2.0)
            if w is None or not (20.0 <= w <= 105.0):
                raise RuntimeError(f'뚜껑 파지 실패(폭 {w}mm)')
            msg(f'🧢 ③ 뚜껑 들어올려 보관대로(P12→P5→P9→P8 · 블렌딩) · 폭 {w:.1f}mm')
            goto(P['lid_mount_pre'], move='lidopen_near')   # 수직 이탈(블렌딩 금지)
            # P12 → P5 → P9 → P8 : 허브·경유지에서 감속 없이 통과
            blend_chain([(common, 'lidopen_travel', cfg.BLEND.get('lead_deg_hub', 20.0)),
                         (P['lid_via'], 'lidopen_travel'),
                         (P['lid_grab'], 'lidopen_near')])
            _rf = cfg.GRIPPER.get('lid_release_force', 15.0)
            msg(f'🧢 ④ 뚜껑 내려놓기(110mm·{_rf:.0f}N)')
            # ★여기서 P9·P5로 복귀하지 않는다. 그 경로는 첫 파지 사이클이 자기 체인 앞에
            #   이어붙여 [P9 → P5 → P2 → P20] 한 번의 연속 이동으로 실행한다.
            #   예전에는 P5에서 한 번 정지 → run_cycle 재시작 → 다시 출발이라 눈에 띄게 섰다.
            grip_set(110, _rf)
            time.sleep(0.5)
            msg('🧢 뚜껑 열기 완료 — 파지 사이클로 연속 진입')
            return {'status': 'ok', 'at': 'lid_grab'}

        # ── phase == 'close' ────────────────────────────────────────────────
        msg('🧢 뚜껑 닫기 ① P5→P9→P8 뚜껑 잡기(10N · 무정지)')
        grip_set(110, 20)          # 개방은 이동과 병행 — P8 도착 전까지 충분히 열린다
        # 포장 완료 시 로봇은 이미 P5 — 가드가 P5 스텝을 지워 P9→P8만 이어 실행된다.
        # 다른 위치에서 시작하면 P5(30°)부터 무정지로 통과한다.
        blend_chain([(common, 'lidclose_travel', cfg.BLEND.get('lead_deg_hub', 30.0)),
                     (P['lid_via'], 'lidclose_travel'),
                     (P['lid_grab'], 'lidclose_near')])
        w = grip_to(0, 10, 2.0)
        if w is None or not (20.0 <= w <= 105.0):
            raise RuntimeError(f'뚜껑 파지 실패(폭 {w}mm)')
        msg(f'🧢 ② 뚜껑 이송(P8→P9→P5 블렌딩 → P12 정밀) · 폭 {w:.1f}mm')
        # P9·P5 를 블렌딩으로 통과해 P12까지 감속 없이 이송한다.
        # P12는 체인의 최종점이라 radius=0 + 도착 검증이 걸려 순응 하강 출발점 정확도는 유지된다.
        blend_chain([(P['lid_via'], 'lidclose_near'),
                     (common, 'lidclose_travel', cfg.BLEND.get('lead_deg_hub', 20.0)),
                     (P['lid_mount_pre'], 'lidclose_travel')])
        # ★순응제어 중 movej는 두산 제약으로 거부됨 → 순응 구간은 movel(posx)로만 이동
        mount_wp = _find_named_wp(wps, cfg.LID_WP_NAMES['lid_mount'])
        locked_wp = _find_named_wp(wps, cfg.LID_WP_NAMES['lid_mount_locked'])
        mount_px = (mount_wp or {}).get('posx')
        locked_px = (locked_wp or {}).get('posx')
        if not (mount_px and locked_px and len(mount_px) == 6 and len(locked_px) == 6):
            raise RuntimeError('P10/P11의 posx 미티칭 — 순응 하강 불가(재티칭 필요)')
        # ★하강량을 설정으로 지정하면 P12 기준으로 그 값만큼만 내려간다.
        #   티칭 좌표(T10/T11)는 X·Y·자세를 그대로 쓰고 Z만 대체 → 순수 Z 하강 유지.
        #   회전 잠금(T11)도 같은 높이에서 이뤄져야 하므로 함께 맞춘다.
        _dz = cfg.GRIPPER.get('lid_descend_mm')
        pre_wp = _find_named_wp(wps, cfg.LID_WP_NAMES['lid_mount_pre'])
        pre_px = (pre_wp or {}).get('posx')
        if _dz is not None and pre_px and len(pre_px) == 6:
            _z = float(pre_px[2]) - float(_dz)
            mount_px = list(mount_px); mount_px[2] = _z
            locked_px = list(locked_px); locked_px[2] = _z
            msg(f'🧢 순응 하강량 {float(_dz):.1f}mm (Z {pre_px[2]:.1f} → {_z:.1f})')
        msg('🧢 ③ 순응제어 ON → P10 안착(직선 하강 · 접촉 허용)')
        comp_on()
        try:
            _v_desc = cfg.move_vel('lid_comp_descend')        # 순응 하강 mm/s(속도탭)
            movel_abs(mount_px, vmm=_v_desc, vdeg=10.0)       # P12→P10: 순수 Z −18mm 하강
            msg('🧢 ④ 회전 잠금(P10→P11 · C축 −41°)')
            _v_rot = cfg.move_vel('lid_comp_rotate')          # 회전 잠금 deg/s(속도탭)
            movel_abs(locked_px, vmm=_v_desc, vdeg=_v_rot)    # C축 회전 체결(순응 유지)
        finally:
            comp_off()                                        # 들어올리기 전 강체 복귀(무게 정확)
        # ⑤ 영점(fz0) — 그리퍼를 완전히 열어 '무부하'를 확정한 뒤 측정한다.
        #    닫은 채로 재면 뚜껑 하중 일부가 그리퍼에 걸려 Δ에서 빠지고, 그 비율이
        #    매번 달라 무게가 튄다. 뚜껑은 이미 통에 잠겨 있어 열어도 떨어지지 않는다.
        if cfg.GRIPPER.get('weight_tare_open', True):
            msg('🧢 ⑤ 영점 측정 — 그리퍼 개방(무부하)')
            grip_to(110, cfg.GRIPPER.get('lid_release_force', 15.0), 1.2)
            wait_fz_settle()
            fz0, sd0 = sample_fz()
            msg('🧢 ⑤b 뚜껑 재파지')
            _rw = grip_to(0, cfg.GRIPPER.get('weight_regrip_force', 10.0), 2.0)
            if _rw is None or not (20.0 <= _rw <= 105.0):
                raise RuntimeError(f'영점 후 뚜껑 재파지 실패(폭 {_rw}mm)')
            wait_fz_settle()                   # 재파지 충격이 잦아든 뒤 들어올린다
        else:
            msg('🧢 ⑤ 무게 기준값 측정(들기 전 · 안정 대기)')
            wait_fz_settle()                   # 순응 해제 직후 잔진동이 잦아들 때까지
            fz0, sd0 = sample_fz()
        msg('🧢 ⑥ 통 들어올리기(+Z 100mm · 잠긴 뚜껑으로 통째)')
        _v_lift = cfg.move_vel('lid_box_lift')                # 통 들기/내리기 mm/s(속도탭)
        movel_z(+100.0, _v_lift)
        # ★고정 대기 대신 '실제로 잠잠해질 때까지' — 통이 뚜껑에 매달려 흔들리는
        #   진자 운동이 남은 채 측정하면 매번 값이 튄다(노이즈의 주원인).
        msg('🧢 ⑥ 진동 안정 대기')
        _settle = wait_fz_settle()
        fz1, sd1 = sample_fz()
        weight = (round(abs(fz1 - fz0) / 9.81, 3)
                  if (fz0 is not None and fz1 is not None) else None)
        # 측정 신뢰도: 두 구간 표준편차를 무게 단위로 환산(±kg)
        w_sd = (round(((sd0 or 0) ** 2 + (sd1 or 0) ** 2) ** 0.5 / 9.81, 3)
                if (sd0 is not None and sd1 is not None) else None)
        if _settle is None:
            msg('⚠️ 진동이 상한 시간 내에 안 잦아듦 — 무게 신뢰도 낮음')
        # ⑦ 들어올린 100mm 중 90mm만 내려온다 → 원래 자리보다 10mm 높은 곳에서 손을 뗀다.
        #    movel_z는 sync_type=0(SYNC)이라 반환 시점이 곧 '이동 완료' 시점이고
        #    내부에서 robot_state까지 확인하므로, 개방은 반드시 정지가 끝난 뒤에 실행된다.
        msg(f'🧢 ⑦ 통 무게 {weight if weight is not None else "?"}kg — 90mm 하강')
        movel_z(-90.0, max(1.0, _v_lift * 0.75))
        time.sleep(0.5)
        _rf = cfg.GRIPPER.get('lid_release_force', 15.0)
        msg(f'🧢 ⑧ 그리퍼 개방({_rf:.0f}N) — 통 놓기')
        grip_to(110, _rf, 1.8)                                # 부드럽게 벌려 손 떼기
        goto(P['lid_mount_pre'], move='lidclose_near')
        goto(common, move='lidclose_travel')
        msg(f'🧢 뚜껑 잠금+무게 검증 완료 · {weight if weight is not None else "?"}kg'
            + (f' ±{w_sd:.3f}' if w_sd is not None else ''))
        return {'status': 'ok', 'weight_kg': weight, 'weight_sd': w_sd,
                'settle_s': _settle, 'fz_before': fz0, 'fz_after': fz1}

    except CycleStop:
        return {'status': 'stopped'}
    except Exception as e:
        msg(f'⚠️ 뚜껑 시퀀스 오류: {e}')
        return {'status': 'error', 'error': str(e)}
