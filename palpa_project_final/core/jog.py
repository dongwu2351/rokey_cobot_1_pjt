"""JogWorker — 로봇 모션 스레드(조그·주문사이클·블록실행·폐기물)."""
# 자동 분리: grip_web.py → core/ (내용 동일, 위치만 이동)

import math
import threading
import time
from types import SimpleNamespace

import grip_config as cfg
import grip_cycle
from core import runtime as RT
from core.runtime import MOTION_ACTIVE, RCLPY_LOCK, STATE_NAMES, JOG_SPEED
from core.runtime import WAYPOINTS, WP_LOCK, PROGRAMS, PROG_LOCK
from core.rg2 import MAX_WIDTH_MM


class JogWorker(threading.Thread):
    """로봇 이동 전용 스레드(자체 rclpy 노드).
    - 드래그 슬라이더: request_goto('j'|'l', pos6, vel) → movej/movel 절대이동(async)
    - 즉시 정지: request_motion_stop() → move_stop
    """
    def __init__(self):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._goto = None          # ('j'|'l', pos6, vel)
        self._movetest = None      # 속도탭 ▶이동: (key, label, src_posj|None, dst_posj, vel)
        self._waste_req = False    # 속도탭 '폐기물 처리' 수동 실행 요청
        self.batch_defect = 0      # 이번 배치에서 나온 불량(구멍) 개수
        self._prog = None          # 블록코딩 프로그램: [{t,...},...] 평면 리스트
        self._prog_i = 0           # 실행된 블록 수(진행표시)
        self._prog_total = 0
        self._prog_msg = ''
        self._prog_vel = 25.0
        self._orders = []          # 주문 [{item,count,packed},...] 다품목
        self._motion_stop = False
        self._fd_req = None        # 자유이동 요청 True/False/None
        self._fd_on = False
        self._fd_origin = None     # 복귀할 원래 관절자세
        self._fd_prev = None
        self._fd_last_move = 0.0
        self._fd_pushed = False
        self._cycle_req = False     # 자동작업 사이클 요청
        self._cycle_stop = False    # 작업 중단 요청
        self._cycle_vel = cfg.DEFAULT_WORK_VEL  # 작업 이동속도
        self._batch_target = 1      # 목표 포장 개수(불량은 미포함)
        self._target_item = 'tennis_normal'  # 이번 배치가 찾는 정확한 주문 등급
        self._order_sequence = ['tennis_normal']  # 다품목 주문 탐색 순서
        self._order_remaining = {'tennis_normal': 1}  # SKU별 남은 포장수
        self._order_requested = {}  # UI/완료요약용 최초 주문(실제 주문 수신 전에는 비움)
        self._batch_attempts = 0    # 판정한 공 수(포장/불량/슬롯반환 포함)
        self._batch_completed = False
        self._batch_weight_kg = None
        self._batch_weight_sd = None
        self.last_cycle_result = {} # 마지막 물리 사이클의 구조화된 분기 결과
        self.last_motion = {}       # 마지막 MoveJ/MoveL 안전 감사 정보
        self._cycle_once = False    # True=정확히 1사이클만(주문 단위 처리용)
        self._training_mode = False # 공 자동 수거→사람 라벨→원래 슬롯 반납 반복
        self._training_stop = False # 현재 공을 슬롯에 반납한 뒤 학습 루프 종료
        self._training_source = 'tennis'
        self.batch_packed = 0       # 이번 배치에서 포장 완료된 개수
        self._opspeed_req = None    # 전체 이동속도 %(1~100) — 펜던트 오버라이드처럼 실시간 스케일
        self._speed_opspeed = cfg.DEFAULT_OPERATION_SPEED   # ★속도탭 전역배속(주문 시작 시 적용)
        self.cycle_active = False
        self.cycle_msg = ''
        self._stop = False
        self.ok = False

    def request_cycle(self, vel=cfg.DEFAULT_WORK_VEL, count=1, once=False,
                      target='tennis_normal', targets=None):
        normalized = cfg.normalize_order_target(target)
        remaining = {}
        sequence = []
        if targets:
            try:
                for raw, qty in targets.items():
                    sku = cfg.normalize_order_target(raw)
                    n = int(qty)
                    if sku not in cfg.ORDER_TARGET_LABELS or n < 1:
                        raise ValueError(f'{raw}:{qty}')
                    if sku not in remaining:
                        sequence.append(sku)
                        remaining[sku] = 0
                    remaining[sku] += n
            except (AttributeError, TypeError, ValueError):
                self.cycle_msg = f'⚠️ 다품목 주문 형식 오류: {targets}'
                return False
            total = sum(remaining.values())
            if not 1 <= total <= 3:
                self.cycle_msg = f'⚠️ 다품목 총수량은 1~3개여야 함: {total}'
                return False
            normalized = sequence[0]
            count = total
        elif normalized in cfg.ORDER_TARGET_LABELS:
            n = max(1, min(3, int(count)))
            sequence = [normalized]
            remaining = {normalized: n}
            count = n
        else:
            self.cycle_msg = f'⚠️ 지원하지 않는 주문 품목: {target}'
            return False
        self._batch_target = int(count)
        self._target_item = normalized
        self._order_sequence = sequence
        self._order_remaining = remaining
        self._order_requested = dict(remaining)
        self._batch_completed = False
        self._batch_weight_kg = None
        self._batch_weight_sd = None
        self._cycle_once = bool(once)
        self._training_mode = False
        self._cycle_vel = max(cfg.SPEED['min_vel'], min(cfg.SPEED['max_vel'], float(vel)))
        self._cycle_stop = False
        self._cycle_req = True
        return True

    def request_training(self, source='tennis', vel=cfg.DEFAULT_WORK_VEL):
        """테니스 P20/야구 P3에서 측정하고 사람 라벨 후 같은 위치에 놓는 빠른 무한루프."""
        source = str(source).strip().lower()
        if source not in ('tennis', 'baseball'):
            self.cycle_msg = f'⚠️ 지원하지 않는 학습 슬롯: {source}'
            return False
        if self.cycle_active or self._cycle_req:
            return False
        self._training_source = source
        self._target_item = 'tennis_normal' if source == 'tennis' else 'baseball_hard'
        self._order_sequence = [self._target_item]
        self._order_remaining = {self._target_item: 1}
        self._batch_target = 1
        self._cycle_once = False
        self._training_mode = True
        self._training_stop = False
        self._cycle_stop = False
        self._cycle_vel = max(cfg.SPEED['min_vel'], min(cfg.SPEED['max_vel'], float(vel)))
        self._cycle_req = True
        return True

    def request_training_stop(self):
        """비상정지가 아니라 현재 공을 측정 위치에 놓고 P5 복귀 후 종료."""
        self._training_stop = True

    def request_cycle_stop(self):
        self._cycle_stop = True

    def request_goto(self, kind, pos6, vel):
        if self.cycle_active or self._cycle_req:
            return False
        with self._lock:
            self._goto = (kind, [float(x) for x in pos6], float(vel))
        return True

    def request_waste(self):
        """폐기물 처리 시퀀스 수동 실행(속도탭 버튼)."""
        if self.cycle_active or self._cycle_req or self._prog:
            return False
        self._waste_req = True
        return True

    def request_move_test(self, key, label, src, dst, vel):
        """'속도' 탭 ▶이동 — 해당 구간을 실제로 재현한다.
        src가 있으면 먼저 출발점으로 안전속도로 이동(무대 세팅)한 뒤,
        출발→도착 구간만 설정된 속도 그대로(1:1) 실행해 체감할 수 있게 한다."""
        if self.cycle_active or self._cycle_req or self._prog:
            return False
        with self._lock:
            self._movetest = (key, label,
                              list(src) if src else None,
                              [float(x) for x in dst], float(vel))
        return True

    def request_prog(self, blocks, vel):
        """블록코딩 프로그램 실행. blocks=평면 리스트 [{t:'move'|'grip'|'measure'|'loop'|'endloop'|'if'|'else'|'endif', ...}]."""
        if self.cycle_active or self._cycle_req:
            return False
        with self._lock:
            self._prog = list(blocks)
            self._prog_i = 0
            self._prog_total = len(blocks)
            self._prog_msg = ''
            self._prog_vel = max(cfg.SPEED['min_vel'], min(cfg.SPEED['max_vel'], float(vel)))
        return bool(self._prog)

    def request_motion_stop(self):
        self._motion_stop = True

    def request_opspeed(self, pct):
        self._opspeed_req = max(1, min(cfg.MAX_OPERATION_SPEED, int(pct)))

    def request_freedrive(self, on):
        self._fd_req = bool(on)

    def stop(self):
        self._stop = True

    def run(self):
        try:
            import rclpy
            from dsr_msgs2.srv import (MoveJoint, MoveLine, MoveStop, SetSingularityHandling,
                                       SetRobotControl, ChangeCollisionSensitivity, SetSafetyMode,
                                       SetRobotMode, GetRobotMode, GetLastAlarm,
                                       GetToolForce, TaskComplianceCtrl, ReleaseComplianceCtrl,
                                       StopRtControl, StartRtControl,
                                       GetRobotState, CheckMotion, MoveSplineJoint,
                                       ChangeOperationSpeed, GetCurrentTool, GetCurrentTcp)
            from controller_manager_msgs.srv import SwitchController
            from std_msgs.msg import Bool, Float64MultiArray
        except Exception:
            return
        with RCLPY_LOCK:
            if not rclpy.ok():
                rclpy.init()
        node = rclpy.create_node('grip_web_jog', namespace='dsr01')
        cmj = node.create_client(MoveJoint, 'motion/move_joint')
        cml = node.create_client(MoveLine, 'motion/move_line')
        cstop = node.create_client(MoveStop, 'motion/move_stop')
        csing = node.create_client(SetSingularityHandling, 'motion/set_singularity_handling')
        cctrl = node.create_client(SetRobotControl, 'system/set_robot_control')
        ccol = node.create_client(ChangeCollisionSensitivity, 'system/change_collision_sensitivity')
        cstate = node.create_client(GetRobotState, 'system/get_robot_state')      # 상태인지 복구용(저빈도)
        cmode = node.create_client(SetRobotMode, 'system/set_robot_mode')          # ★자율모드 복원용
        cgmode = node.create_client(GetRobotMode, 'system/get_robot_mode')
        calarm = node.create_client(GetLastAlarm, 'system/get_last_alarm')   # ★실패 원인 코드
        cchk = node.create_client(CheckMotion, 'motion/check_motion')             # 이동완료 판정 가속
        cspl = node.create_client(MoveSplineJoint, 'motion/move_spline_joint')    # 연속 이송(movesj)
        cops = node.create_client(ChangeOperationSpeed, 'motion/change_operation_speed')  # 전체 속도 %
        cget_tool = node.create_client(GetCurrentTool, 'tool/get_current_tool')
        cget_tcp = node.create_client(GetCurrentTcp, 'tcp/get_current_tcp')
        csafe = node.create_client(SetSafetyMode, 'system/set_safety_mode')
        ccomp_on = node.create_client(TaskComplianceCtrl, 'force/task_compliance_ctrl')
        ccomp_off = node.create_client(ReleaseComplianceCtrl, 'force/release_compliance_ctrl')
        ctf = node.create_client(GetToolForce, 'aux_control/get_tool_force')   # 뚜껑 무게측정용(직접 호출)
        cswitch = node.create_client(SwitchController, 'controller_manager/switch_controller')
        crt_stop = node.create_client(StopRtControl, 'realtime/stop_rt_control')
        crt_start = node.create_client(StartRtControl, 'realtime/start_rt_control')
        hg_pub = node.create_publisher(Bool, 'hand_guiding', 1)   # 하드웨어 티칭모드 토글
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        self.ok = True

        def call(cli, req, cto=1.5):
            try:
                if not cli.wait_for_service(timeout_sec=0.3):
                    return None
                fut = cli.call_async(req)
                executor.spin_until_future_complete(fut, timeout_sec=cto)
                return fut.result()
            except Exception:
                return None

        # ── 블록코딩 인터프리터 (move/grip/measure/loop/if) ──────────────────────
        def last_alarm():
            """마지막 알람 코드 반환 'group.index' (5.7060=충돌 / 5.7170=잘못된모드 / 5.7173=범위초과 등)."""
            try:
                r = call(calarm, GetLastAlarm.Request(), cto=1.0)
                la = getattr(r, 'log_alarm', None)
                if la is not None:
                    return f'{la.group}.{la.index}'
            except Exception:
                pass
            return '?'

        def ensure_auto():
            """★movej는 자율모드(AUTONOMOUS=1)에서만 허용. 아니면 알람 5.7170(INVALID_SYSTEM_STATE)로
            거부되어 '이동이 안 되고 걸린 것처럼' 보인다. 복구 후 RECOVERY/MANUAL에 남는 경우가 많아
            이동 전·복구 후 반드시 자율모드로 복원한다."""
            r = call(cgmode, GetRobotMode.Request(), cto=1.0)
            md = getattr(r, 'robot_mode', -1) if r else -1
            if md == 1:
                return True
            for _ in range(3):
                call(cmode, SetRobotMode.Request(robot_mode=1), cto=1.5)
                time.sleep(0.25)
                r = call(cgmode, GetRobotMode.Request(), cto=1.0)
                if r is not None and getattr(r, 'robot_mode', -1) == 1:
                    return True
            return False

        def set_collision(v):
            """MANUAL 모드에서만 호출하는 충돌감도 실제 응답 확인."""
            v = int(v)
            if not 1 <= v <= 100:
                return False
            for _ in range(3):
                r = call(ccol, ChangeCollisionSensitivity.Request(sensitivity=int(v)), cto=1.5)
                if r is not None and getattr(r, 'success', False):
                    return True
                time.sleep(0.3)
            return False

        def soft_stop():
            rq = MoveStop.Request()
            rq.stop_mode = 2
            return call(cstop, rq, cto=2.0)

        def fresh_joints():
            if not RT.JOINTS:
                return None
            pos, sample_ts = RT.JOINTS.snapshot()
            if (not pos or len(pos) != 6 or not sample_ts
                    or time.time() - sample_ts > cfg.JOINT_SAMPLE_MAX_AGE_S):
                return None
            return [float(x) for x in pos]

        def robot_state():
            r = call(cstate, GetRobotState.Request(), cto=1.0)
            return getattr(r, 'robot_state', -1) if r else -1

        def motion_status():
            r = call(cchk, CheckMotion.Request(), cto=0.8)
            if r is None or not getattr(r, 'success', False):
                return None
            return int(r.status)

        def prepare_collision_for_motion(v):
            """실기 제어기 규칙에 맞춰 정지 상태에서 충돌감도를 한 번 설정한다.

            AUTONOMOUS에서 change_collision_sensitivity를 호출하면 5.7170으로
            거부되므로 MANUAL로 전환해 설정한 뒤, 성공/실패와 무관하게 반드시
            AUTONOMOUS로 복귀한다. 실제 이동 중에는 이 함수를 호출하지 않는다.
            """
            v = int(v)
            if not 1 <= v <= 100:
                return False
            if robot_state() != 1 or motion_status() != 0:
                return False

            manual_ok = False
            configured = False
            auto_ok = False
            try:
                rm = call(cgmode, GetRobotMode.Request(), cto=1.0)
                manual_ok = rm is not None and getattr(rm, 'robot_mode', -1) == 0
                if not manual_ok:
                    for _ in range(3):
                        r = call(cmode, SetRobotMode.Request(robot_mode=0), cto=1.5)
                        time.sleep(0.25)
                        rm = call(cgmode, GetRobotMode.Request(), cto=1.0)
                        if (r is not None and getattr(r, 'success', False)
                                and rm is not None and getattr(rm, 'robot_mode', -1) == 0):
                            manual_ok = True
                            break
                if manual_ok:
                    configured = set_collision(v)
            finally:
                # 설정 실패 시에도 MANUAL에 방치하지 않는다.
                for _ in range(3):
                    r = call(cmode, SetRobotMode.Request(robot_mode=1), cto=1.5)
                    time.sleep(0.25)
                    rm = call(cgmode, GetRobotMode.Request(), cto=1.0)
                    if (r is not None and getattr(r, 'success', False)
                            and rm is not None and getattr(rm, 'robot_mode', -1) == 1):
                        auto_ok = True
                        break
            return (manual_ok and configured and auto_ok
                    and robot_state() == 1 and motion_status() == 0)

        def assert_safe_state(st):
            if st in (3, 5, 8, 9, 10):
                raise RuntimeError(
                    f'안전정지 상태({st}, 알람 {last_alarm()}) — 자동복구/재이동 금지')
            if st in (6, 15):
                raise RuntimeError(f'복구 금지 로봇 상태({st}, 알람 {last_alarm()})')

        def live_tool_snapshot():
            rt = call(cget_tool, GetCurrentTool.Request(), cto=1.0)
            rc = call(cget_tcp, GetCurrentTcp.Request(), cto=1.0)
            if (rt is None or rc is None or not getattr(rt, 'success', False)
                    or not getattr(rc, 'success', False)):
                return None, None, 0.0
            return str(rt.info), str(rc.info), time.time()

        def tool_ready():
            tool, tcp, sample_ts = live_tool_snapshot()
            if not sample_ts:
                return False, '공구/TCP 실시간 조회 실패'
            if tool != cfg.REQUIRED_TOOL or tcp != cfg.REQUIRED_TCP:
                return False, (
                    f'공구/TCP 불일치(필요 {cfg.REQUIRED_TOOL}/{cfg.REQUIRED_TCP}, '
                    f'현재 {tool}/{tcp})')
            return True, ''

        def motion_preflight():
            cur = fresh_joints()
            if cur is None:
                raise RuntimeError('최신 관절 토픽 없음')
            st = robot_state()
            assert_safe_state(st)
            if st != 1 or motion_status() != 0:
                raise RuntimeError(f'이동 전 로봇이 STANDBY/IDLE 아님(state={st})')
            ok, reason = tool_ready()
            if not ok:
                raise RuntimeError(reason)
            if not ensure_auto():
                raise RuntimeError('AUTONOMOUS 모드 전환 실패')
            return cur

        def assert_idle_ready():
            st = robot_state()
            assert_safe_state(st)
            if st != 1 or motion_status() != 0:
                raise RuntimeError(f'다음 이동 전 STANDBY/IDLE 아님(state={st})')

        def motion_values(vel, profile):
            vel = max(cfg.SPEED['min_vel'], float(vel))
            vmax = cfg.SPEED.get(profile + '_max')
            if vmax:
                vel = min(vel, float(vmax))
            ratio = cfg.SPEED.get(profile + '_acc_ratio', cfg.SPEED['acc_ratio'])
            acc = max(cfg.SPEED.get('min_acc', 5.0), vel * ratio)
            amax = cfg.SPEED.get(profile + '_acc_max')
            if amax:
                acc = min(acc, float(amax))
            return float(vel), float(acc)

        def audit_motion(kind, target, current, vel, acc, profile, status='sent'):
            self.last_motion = {
                'kind': kind,
                'stage': self._prog_msg or self.cycle_msg,
                'current': [round(float(x), 3) for x in current] if current else None,
                'target': [round(float(x), 3) for x in target] if target else None,
                'vel': round(float(vel), 3),
                'acc': round(float(acc), 3),
                'profile': profile,
                'time': time.strftime('%H:%M:%S'),
                'status': status,
            }

        def wait_joint_done(target, start, vel):
            travel = max(abs(target[i] - start[i]) for i in range(6))
            timeout = max(45.0, min(180.0, travel / max(vel, 1.0) * 12.0 + 15.0))
            t0 = time.time()
            missing_since = None
            prev = None
            near_count = idle_count = stopped_off_target = 0
            state_misses = 0
            last_state_check = 0.0
            while time.time() - t0 < timeout:
                if self._motion_stop:
                    soft_stop()
                    raise RuntimeError('사용자 이동 중단')
                cur = fresh_joints()
                if cur is None:
                    missing_since = missing_since or time.time()
                    if time.time() - missing_since >= cfg.JOINT_SAMPLE_MAX_AGE_S:
                        soft_stop()
                        raise RuntimeError('이동 중 관절 토픽 갱신 중단')
                    time.sleep(0.1)
                    continue
                missing_since = None
                if time.time() - last_state_check >= 0.5:
                    last_state_check = time.time()
                    st = robot_state()
                    if st < 0:
                        state_misses += 1
                        if state_misses >= 3:
                            soft_stop()
                            raise RuntimeError('이동 중 로봇 상태 조회 실패')
                    else:
                        state_misses = 0
                        assert_safe_state(st)
                near = max(abs(cur[i] - target[i]) for i in range(6)) < cfg.MOTION['near_deg']
                still = prev is not None and max(
                    abs(cur[i] - prev[i]) for i in range(6)) < cfg.MOTION['still_deg']
                if near and still:
                    near_count += 1
                    if near_count >= cfg.MOTION['confirm_n']:
                        st = robot_state()
                        assert_safe_state(st)
                        if motion_status() == 0 and st == 1:
                            idle_count += 1
                            if idle_count >= cfg.MOTION['idle_confirm']:
                                self.last_motion['status'] = 'arrived'
                                return True
                        else:
                            idle_count = 0
                elif still:
                    stopped_off_target += 1
                    if stopped_off_target >= 6 and motion_status() == 0:
                        st = robot_state()
                        assert_safe_state(st)
                        raise RuntimeError('MoveJ가 목표 밖에서 정지')
                else:
                    near_count = idle_count = stopped_off_target = 0
                prev = cur
                time.sleep(cfg.MOTION['poll_s'])
            soft_stop()
            raise RuntimeError(f'MoveJ 도착 시간초과({int(timeout)}s)')

        def wait_motion_done(timeout=90.0):
            t0 = time.time()
            idle_count = misses = 0
            while time.time() - t0 < timeout:
                if self._motion_stop:
                    soft_stop()
                    raise RuntimeError('사용자 이동 중단')
                st = robot_state()
                if st < 0:
                    misses += 1
                    if misses >= 3:
                        soft_stop()
                        raise RuntimeError('MoveL 중 로봇 상태 조회 실패')
                else:
                    misses = 0
                    assert_safe_state(st)
                status = motion_status()
                if time.time() - t0 >= 0.3 and status == 0 and st == 1:
                    idle_count += 1
                    if idle_count >= cfg.MOTION['idle_confirm']:
                        self.last_motion['status'] = 'arrived'
                        return True
                else:
                    idle_count = 0
                time.sleep(cfg.MOTION['poll_s'])
            soft_stop()
            raise RuntimeError(f'MoveL 완료 시간초과({int(timeout)}s)')

        def _home_posj():
            # 홈 = 티칭에서 역할'홈'으로 저장한 웨이포인트 우선, 없으면 config 기본값.
            with WP_LOCK:
                for w in WAYPOINTS:
                    if w.get('role') == 'home' and w.get('posj') and len(w['posj']) == 6:
                        return list(w['posj'])
            return list(cfg.HOME_POSJ)

        def _p_goto(tgt, vel, radius=0.0, sync=1):
            # dsr_controller2의 비동기 MoveJ는 radius를 전달하지 않는다. 블록의 radius/sync
            # 값과 무관하게 각 절대목표에서 완전히 정지한 후 다음 명령을 보낸다.
            _ = radius, sync
            cur = fresh_joints()
            if not (tgt and len(tgt) == 6 and cur and len(cur) == 6):
                raise RuntimeError('MoveJ 목표/최신 관절값 없음')
            assert_idle_ready()
            vel, acc = motion_values(vel, 'program')
            if max(abs(cur[i] - tgt[i]) for i in range(6)) < 0.2:
                if robot_state() != 1 or motion_status() != 0:
                    raise RuntimeError('목표 근처지만 STANDBY/IDLE 상태가 아님')
                return True
            req = MoveJoint.Request()
            req.pos = [float(x) for x in tgt]
            req.vel = vel; req.acc = acc
            req.time = 0.0; req.mode = 0; req.blend_type = 0; req.sync_type = 1
            req.radius = 0.0
            audit_motion('MoveJ', tgt, cur, vel, acc, 'program')
            res = call(cmj, req, cto=5.0)
            if res is None or not getattr(res, 'success', False):
                self.last_motion['status'] = 'rejected'
                self._prog_msg = f'이동 거부 — 알람 {last_alarm()}'
                raise RuntimeError(self._prog_msg)
            return wait_joint_done(tgt, cur, vel)

        def _p_movel(ax, d, vel):
            """베이스축 상대 MoveL도 비동기 1회 전송 후 IDLE/STANDBY를 확인한다."""
            delta = [0.0] * 6
            assert_idle_ready()
            try:
                delta[int(ax)] = float(d)
            except (ValueError, IndexError):
                raise RuntimeError('MoveL 축/거리 형식 오류')
            vel, acc = motion_values(vel, 'program')
            req = MoveLine.Request()
            req.pos = delta; req.vel = [vel * 2.5, vel]; req.acc = [acc * 2.5, acc]
            req.time = 0.0; req.radius = 0.0; req.ref = 0
            req.mode = 1; req.blend_type = 0; req.sync_type = 1
            audit_motion('MoveL', delta, fresh_joints(), vel, acc, 'program')
            res = call(cml, req, cto=5.0)
            if res is None or not getattr(res, 'success', False):
                self.last_motion['status'] = 'rejected'
                raise RuntimeError(f'MoveL 거부 — 알람 {last_alarm()}')
            return wait_motion_done()

        def _p_movej1(j, deg, vel):
            """최신 관절값으로 절대목표를 만든 뒤 안전한 MoveJ로 실행한다."""
            cur = fresh_joints()
            if cur is None:
                raise RuntimeError('MoveJ1 최신 관절값 없음')
            assert_idle_ready()
            target = list(cur)
            try:
                target[int(j)] += float(deg)
            except (ValueError, IndexError):
                raise RuntimeError('MoveJ1 관절/각도 형식 오류')
            vel, acc = motion_values(vel, 'program')
            req = MoveJoint.Request()
            req.pos = target; req.vel = vel; req.acc = acc
            req.time = 0.0; req.radius = 0.0
            req.mode = 0; req.blend_type = 0; req.sync_type = 1
            audit_motion('MoveJ', target, cur, vel, acc, 'program')
            res = call(cmj, req, cto=5.0)
            if res is None or not getattr(res, 'success', False):
                self.last_motion['status'] = 'rejected'
                raise RuntimeError(f'MoveJ1 거부 — 알람 {last_alarm()}')
            return wait_joint_done(target, cur, vel)

        def _p_grip(width, force):
            if RT.STATE:
                RT.STATE.set_force(force); RT.STATE.set_target(width); time.sleep(1.3)

        def _p_eval(cond, cls):
            if isinstance(cond, str):
                return bool(cond) and cond in cls
            terms = cond.get('terms', []) if isinstance(cond, dict) else []
            op = cond.get('op', 'or') if isinstance(cond, dict) else 'or'

            def m(t):
                if t in ('못잡음', '탐지안됨', 'notgrip'):
                    return ('못' in cls) or ('없음' in cls) or ('오류' in cls)
                if t == '주문필요':   # 지금 판별된 공이 '아직 더 담아야 할 주문품'인가
                    return any((o['item'] in cls) and o['packed'] < o['count'] for o in self._orders)
                if t == '주문완료':   # 모든 주문 항목 충족?
                    return bool(self._orders) and all(o['packed'] >= o['count'] for o in self._orders)
                return t in cls
            if not terms:
                return False
            return all(m(t) for t in terms) if op == 'and' else any(m(t) for t in terms)

        def _p_parse(flat, i, stops):
            nodes = []
            while i < len(flat):
                b = flat[i]; t = b.get('t')
                if t in stops:
                    return nodes, i
                if t == 'loop':
                    body, i = _p_parse(flat, i + 1, ('endloop',))
                    nodes.append({'t': 'loop', 'count': max(1, int(b.get('count', 1) or 1)), 'body': body})
                    i += 1
                elif t == 'if':
                    thenb, i = _p_parse(flat, i + 1, ('else', 'endif'))
                    elseb = []
                    if i < len(flat) and flat[i].get('t') == 'else':
                        elseb, i = _p_parse(flat, i + 1, ('endif',))
                    nodes.append({'t': 'if', 'cond': b.get('cond', ''), 'then': thenb, 'else': elseb})
                    i += 1
                elif t in ('endloop', 'endif', 'else'):
                    i += 1
                else:
                    nodes.append(b); i += 1
            return nodes, i

        def _p_run(nodes, vel, depth=0):
            for n in nodes:
                if self._motion_stop or self._prog is None:
                    return
                t = n.get('t')
                with self._lock:
                    self._prog_i += 1
                    self._prog_msg = t
                if t == 'move':
                    wid = int(n.get('id', -1))
                    if wid < 0:                       # id<0 = 홈
                        tgt = _home_posj()
                    else:
                        with WP_LOCK:
                            tgt = WAYPOINTS[wid]['posj'] if wid < len(WAYPOINTS) else None
                    _p_goto(tgt, vel, n.get('radius', 0), n.get('sync', 1))
                elif t == 'home':
                    _p_goto(_home_posj(), vel, n.get('radius', 0), n.get('sync', 1))
                elif t == 'movel':                 # 베이스축(직교) 상대이동
                    _p_movel(n.get('ax', 0), n.get('d', 10), vel)
                elif t == 'movej1':                # 단일 관절 상대이동(펜던트식)
                    _p_movej1(n.get('j', 0), n.get('deg', 10), vel)
                elif t == 'order':                 # 주문 항목 추가(여러 개 쌓으면 다품목 주문)
                    it = n.get('item', '하드'); cnt = max(1, int(n.get('count', 1) or 1))
                    with self._lock:
                        for o in self._orders:
                            if o['item'] == it:
                                o['count'] = cnt; o['packed'] = 0; break
                        else:
                            self._orders.append({'item': it, 'count': cnt, 'packed': 0})
                    self._prog_msg = '주문 ' + ' · '.join(f"{o['item']}{o['count']}" for o in self._orders)
                elif t == 'pack':                  # 지금 판별된 공을 '주문 항목'에 1개 반영
                    cls = (RT.STATE.flow_class if RT.STATE else '') or ''
                    for o in self._orders:
                        if o['item'] in cls and o['packed'] < o['count']:
                            o['packed'] += 1; break
                    self._prog_msg = ' · '.join(f"{o['item']} {o['packed']}/{o['count']}" for o in self._orders)
                elif t == 'grip':
                    _p_grip(float(n.get('width', 60)), float(n.get('force', 20)))
                elif t == 'measure':
                    if RT.STATE:
                        self._prog_msg = RT.STATE.grip_classify()
                elif t == 'wait':
                    dur = max(0.0, min(60.0, float(n.get('sec', 1) or 1)))
                    slept = 0.0
                    while slept < dur and not self._motion_stop and self._prog is not None:
                        time.sleep(min(0.2, dur - slept)); slept += 0.2
                elif t == 'loop':
                    if n.get('mode') == 'order':      # ★주문(전 품목) 채울 때까지 반복
                        need = sum(max(0, o['count'] - o['packed']) for o in self._orders)
                        guard = 0
                        while True:
                            if not self._orders or all(o['packed'] >= o['count'] for o in self._orders):
                                break
                            guard += 1
                            if guard > need * 5 + 15:         # 무한루프 방지 상한
                                self._prog_msg = '주문 미달 중단 — ' + ' · '.join(
                                    f"{o['item']} {o['packed']}/{o['count']}" for o in self._orders)
                                break
                            if self._motion_stop or self._prog is None:
                                return
                            _p_run(n['body'], vel, depth)
                    else:
                        for _ in range(n['count']):
                            if self._motion_stop or self._prog is None:
                                return
                            _p_run(n['body'], vel, depth)
                elif t == 'if':
                    cls = (RT.STATE.flow_class if RT.STATE else '') or ''
                    _p_run(n['then'] if _p_eval(n['cond'], cls) else n['else'], vel, depth)
                elif t == 'call':                 # ★함수 호출: 저장된 시퀀스 실행
                    sub = PROGRAMS.get(n.get('name', ''))
                    if sub is not None and depth < 12:      # 무한재귀 방지
                        subtree, _ = _p_parse(sub, 0, ())
                        _p_run(subtree, vel, depth + 1)

        def exec_prog(flat, vel):
            tree, _ = _p_parse(flat, 0, ())
            with self._lock:
                self._orders = []        # 실행마다 주문 새로 시작
            MOTION_ACTIVE.set()
            try:
                if not prepare_collision_for_motion(cfg.COLLISION['fixed']):
                    raise RuntimeError(
                        f"고정 충돌감도({cfg.COLLISION['fixed']}) 준비 실패")
                motion_preflight()
                _p_run(tree, vel)
            finally:
                MOTION_ACTIVE.clear()

        # 시작 설정: 특이점 자동회피. 충돌감도는 실제 이동 직전 MANUAL에서 설정한다.
        try:
            call(csing, SetSingularityHandling.Request(mode=0), cto=2.0)
        except Exception:
            pass

        def switch_ctrl(activate, deactivate):
            rq = SwitchController.Request()
            rq.activate_controllers = activate
            rq.deactivate_controllers = deactivate
            rq.strictness = 1   # BEST_EFFORT
            rq.activate_asap = True
            try:
                from builtin_interfaces.msg import Duration as _Dur
                rq.timeout = _Dur(sec=2, nanosec=0)
            except Exception:
                pass
            return call(cswitch, rq, cto=3.0)

        # ★ctx(로봇 컨텍스트)는 루프 밖에서 한 번만 만든다.
        #   예전엔 주문 사이클 블록 안에서만 생성해, 사이클을 한 번도 안 돌린 상태로
        #   폐기물 처리를 실행하면 'ctx referenced before assignment'로 죽었다.
        def wp_snap():
            with WP_LOCK:
                return [dict(w) for w in WAYPOINTS]
        def fresh_cycle_joints():
            return fresh_joints()
        ctx = SimpleNamespace(
            jog=self, call=call, cmj=cmj, cstop=cstop, ccol=ccol,
            cctrl=cctrl, cstate=cstate, cchk=cchk, cspl=cspl,
            MoveJoint=MoveJoint, MoveStop=MoveStop,
            ChangeCollisionSensitivity=ChangeCollisionSensitivity,
            SetRobotControl=SetRobotControl, GetRobotState=GetRobotState,
            CheckMotion=CheckMotion, MoveSplineJoint=MoveSplineJoint,
            Float64MultiArray=Float64MultiArray,
            joints=fresh_cycle_joints,
            ensure_auto=ensure_auto,
            prepare_collision=prepare_collision_for_motion,
            tool_snapshot=live_tool_snapshot,
            waypoints=wp_snap, state=RT.STATE, motion_active=MOTION_ACTIVE,
            # 뚜껑 시퀀스용: 직선이동·순응제어·로봇 힘 스냅샷
            cml=cml, MoveLine=MoveLine,
            ccomp_on=ccomp_on, ccomp_off=ccomp_off,
            TaskComplianceCtrl=TaskComplianceCtrl,
            ReleaseComplianceCtrl=ReleaseComplianceCtrl,
            robot=lambda: (RT.ROBOT.snapshot() if RT.ROBOT else {}),
            ctf=ctf, GetToolForce=GetToolForce,
        )

        while not self._stop:
            # ── 티칭모드(핸드가이딩): RT제어 정지 → 로봇 서보홀드 해제 → 백드라이브(밀림). ──
            #    stop_rt_control은 컨트롤러 비활성화(잼)와 달리 정식 서비스라 안전. OFF시 start로 복원.
            if self._opspeed_req is not None:
                v = self._opspeed_req; self._opspeed_req = None
                r = ChangeOperationSpeed.Request(); r.speed = v
                call(cops, r, cto=1.0)   # 사이클 진행 중에도 즉시 반영(전체 %)
            fd = self._fd_req
            if fd is not None:
                self._fd_req = None
                if fd:
                    hg_pub.publish(Bool(data=True))    # 제어권 넘기기(펜던트 요청 시 자동승인)
                    self._fd_on = True
                else:
                    # 회수 복구는 하드웨어 구독콜백이 전담(자율모드+FORCE_REQUEST+상태복구+서보온).
                    # Python은 발행만 → 중복호출로 DRFL 잼 방지.
                    hg_pub.publish(Bool(data=False))
                    time.sleep(1.5)
                    self._fd_on = False
            if self._fd_on:
                time.sleep(0.05)
                continue

            # ── 자동작업 사이클: grip_cycle 모듈(설정=grip_config)로 분리 ──
            if self._cycle_req:
                self._cycle_req = False
                self.cycle_active = True
                self.batch_packed = 0
                self._batch_attempts = 0
                self.last_cycle_result = {}
                fails = 0
                # ★주문 시작 시 전역배속(op_speed)을 저장된 값(기본 100)으로 세팅 → 구간속도 1:1 성립.
                #   (op<100이면 슬라이더값이 그만큼 깎여서 1:1이 안 됨)
                try:
                    _op = int(getattr(self, '_speed_opspeed', cfg.DEFAULT_OPERATION_SPEED))
                    call(cops, ChangeOperationSpeed.Request(speed=max(1, min(100, _op))), cto=1.5)
                except Exception:
                    pass
                try:
                    if self._training_mode:
                        first = True
                        while not self._training_stop:
                            result = grip_cycle.run_cycle(
                                ctx, first=first, home_return=False,
                                target_item=self._target_item, training_mode=True)
                            self.last_cycle_result = result or {}
                            first = False
                            self._batch_attempts += 1
                            status = (result or {}).get('status', 'error')
                            if status == 'trained':
                                self.cycle_msg = (
                                    f'🧪 {self._training_source} 학습 '
                                    f'{self._batch_attempts}회 완료 · 다음 공 시작')
                                time.sleep(0.5)
                                continue
                            if status == 'training_stopped':
                                break
                            detail = (result or {}).get('error') or status
                            self.cycle_msg = (
                                f'⚠️ 로봇 학습 루프 중단 ({self._batch_attempts}회): {detail}')
                            break
                        if (self._training_stop
                                and (self.last_cycle_result or {}).get('status')
                                != 'training_stopped'):
                            self.cycle_msg = (
                                f'🧪 학습 루프 종료 · P5 대기 '
                                f'(저장 {self._batch_attempts}회)')
                        continue
                    if self._cycle_once:
                        self.last_cycle_result = grip_cycle.run_cycle(
                            ctx, first=True, home_return=False, target_item=self._target_item)
                        continue
                    # 주문 전체에서 아직 필요한 SKU면 현재 탐색 목표와 달라도 P6에 넣는다.
                    # 불량/주문에 없는 SKU만 분리·슬롯반환 후 계속 검색한다.
                    first = True
                    batch_outcome = None
                    max_attempts = max(20, self._batch_target * 12)
                    # ★주문 시작: 뚜껑 열기(P5→P12→P10 잡기5N→P12→P5→P9→P8 놓기→P5)
                    # ★배치 전체에서 충돌감도 설정(MANUAL↔AUTONOMOUS 전환)은 여기서 딱 한 번.
                    #   예전엔 뚜껑 열기 뒤 첫 공 사이클이 first=True로 이 전환을 또 수행해
                    #   '파란불 대기'가 생겼다(모드 전환은 정지 상태에서만 가능해 흐름이 끊김).
                    self.cycle_msg = '🛡 충돌감도 설정'
                    if not prepare_collision_for_motion(cfg.COLLISION['fixed']):
                        batch_outcome = (f"⚠️ 고정 충돌감도({cfg.COLLISION['fixed']}) "
                                         '준비 실패 — 배치 중단')
                        self.cycle_msg = batch_outcome
                    ctx.collision_ready = True     # run_cycle이 재설정을 건너뛰게 함
                    self.cycle_msg = '🧢 뚜껑 열기 시작'
                    _lid = grip_cycle.run_lid(ctx, 'open') if batch_outcome is None else None
                    _lid_at = (_lid or {}).get('at')   # 'lid_grab'이면 P8에서 바로 이어받음
                    if batch_outcome is None and (_lid or {}).get('status') != 'ok':
                        batch_outcome = (f"⚠️ 뚜껑 열기 실패 — 배치 중단: "
                                         f"{(_lid or {}).get('error', (_lid or {}).get('status', ''))}")
                        self.cycle_msg = batch_outcome
                    _prev_at_lift = False; _prev_next = None
                    self.batch_defect = 0        # 이번 배치 불량 개수
                    while batch_outcome is None:
                        next_target = next(
                            (sku for sku in self._order_sequence
                             if self._order_remaining.get(sku, 0) > 0),
                            None,
                        )
                        if next_target is None:
                            break
                        self._target_item = next_target
                        # ★이전 사이클이 다음 파지 경유지까지 이어서 데려다 놨고 그 예측이
                        #   지금 목표와 같으면 P5 왕복을 건너뛴다(사이클 경계 제거).
                        _resume = bool(_prev_at_lift and _prev_next == next_target)
                        result = grip_cycle.run_cycle(
                            ctx, first=first, home_return=False,
                            target_item=self._target_item,
                            order_remaining=self._order_remaining,
                            resume_at_lift=_resume,
                            # 뚜껑을 P8에 놓은 직후면 첫 사이클이 P9→P5 복귀까지 이어서 실행
                            start_from_lid=(first and _lid_at == 'lid_grab'))
                        self.last_cycle_result = result or {}
                        _prev_at_lift = bool((result or {}).get('at_pick'))
                        _prev_next = (result or {}).get('next_target')
                        first = False
                        self._batch_attempts += 1
                        status = (result or {}).get('status', 'error')

                        if status == 'defect':
                            self.batch_defect += 1   # 폐기물 처리 실행 여부 판단용
                        if status in ('packed', 'defect', 'rerouted'):
                            fails = 0
                        elif status == 'empty':
                            batch_outcome = (f'🚨 공이 없습니다! 공을 채워주세요 '
                                             f'(포장 {self.batch_packed}/{self._batch_target}에서 중단)')
                            self.cycle_msg = batch_outcome
                            break
                        elif status == 'stopped':
                            break
                        elif status == 'retry':
                            fails += 1
                        else:
                            # 이동/그리퍼/설정 오류 뒤 자동 재시도는 공을 안전하지 않은 위치에
                            # 놓을 수 있으므로 즉시 멈추고 작업자 확인을 요구한다.
                            detail = (result or {}).get('error') or status
                            batch_outcome = (
                                f'⚠️ 사이클 오류 — 배치 중단 '
                                f'(포장 {self.batch_packed}/{self._batch_target}): {detail}')
                            self.cycle_msg = batch_outcome
                            break

                        # P6 개방 직후 run_cycle이 이미 원자적으로 count를 올린다.
                        if self._cycle_stop or self.batch_packed >= self._batch_target:
                            break
                        if fails >= 2:
                            batch_outcome = f'⚠️ 연속 실패 2회 — 배치 중단 (포장 {self.batch_packed}/{self._batch_target})'
                            self.cycle_msg = batch_outcome
                            break
                        if self._batch_attempts >= max_attempts:
                            batch_outcome = (f'⚠️ 검색 상한 {max_attempts}회 — 배치 중단 '
                                             f'(포장 {self.batch_packed}/{self._batch_target})')
                            self.cycle_msg = batch_outcome
                            break
                        remain_text = ' · '.join(
                            f'{cfg.ORDER_TARGET_LABELS.get(s, s)} {n}'
                            for s, n in self._order_remaining.items() if n > 0)
                        self.cycle_msg = (
                            f'📦 전체 포장 {self.batch_packed}/{self._batch_target} '
                            f'· 남음 {remain_text or "없음"} — 다음 공 판정')
                        # ★다음 사이클 전 그리퍼 열림 가드(안 열렸으면 개방 재요청 후 확인)
                        if RT.STATE and RT.STATE.actual_width < 90.0:
                            RT.STATE.request_flow('release'); time.sleep(2.0)
                            if RT.STATE.actual_width < 90.0:
                                batch_outcome = f'⚠️ 그리퍼 미개방(폭 {RT.STATE.actual_width}mm) — 배치 중단'
                                self.cycle_msg = batch_outcome
                                break
                        time.sleep(0.2)   # 사이클 간 최소 간격
                    # 이동/보호정지 오류 후에는 현재 자세가 불명확하므로 P5 자동복귀를
                    # 추가로 명령하지 않는다. 정상 분기들은 run_cycle 내부에서 이미 P5에 있다.
                    safe_to_finish = (
                        not self._cycle_stop
                        and (not self.last_cycle_result
                             or self.last_cycle_result.get('status')
                             in ('packed', 'defect', 'rerouted', 'empty', 'retry')))
                    # ★뚜껑 닫기로 바로 이어지는 경우엔 P5 복귀를 생략한다.
                    #   run_lid('close')가 현재 위치에서 [P5 → P9 → P8] 체인을 바로 시작하므로
                    #   여기서 P5에 한 번 정지시키면 그 사이가 눈에 띄게 비었다.
                    _will_close_lid = (self.batch_packed >= self._batch_target
                                       and not self._cycle_stop and batch_outcome is None)
                    if safe_to_finish and not _will_close_lid:
                        finish_result = grip_cycle.run_cycle(
                            ctx, finish_only=True, target_item=self._target_item)
                        if ((finish_result or {}).get('status') != 'ready'
                                and batch_outcome is None):
                            batch_outcome = (
                                f'⚠️ P5 안전 복귀 실패 — 배치 중단 '
                                f'(포장 {self.batch_packed}/{self._batch_target})')
                    if (self.batch_packed >= self._batch_target
                            and not self._cycle_stop and batch_outcome is None):
                        label = ' · '.join(
                            cfg.ORDER_TARGET_LABELS.get(s, s)
                            for s in self._order_sequence)
                        batch_outcome = (
                            f'🎉 모두 담았습니다! {label} 포장 '
                            f'{self.batch_packed}/{self._batch_target} 완료')
                    elif self._cycle_stop and batch_outcome is None:
                        batch_outcome = f'⏹ 배치 중단됨 (포장 {self.batch_packed}/{self._batch_target})'
                    # ★주문 완료 시: 뚜껑 닫기(P8 잡기10N→P12→순응→P10 안착→P11 잠금)+통 무게 검증
                    if batch_outcome and batch_outcome.startswith('🎉'):
                        self.cycle_msg = '🧢 뚜껑 닫기 + 통 무게 검증 시작'
                        _lid2 = grip_cycle.run_lid(ctx, 'close')
                        if (_lid2 or {}).get('status') == 'ok':
                            _wkg = _lid2.get('weight_kg')
                            self._batch_completed = True
                            self._batch_weight_kg = _wkg
                            _wsd = _lid2.get('weight_sd')
                            self._batch_weight_sd = _wsd
                            batch_outcome += (
                                f' · 🧢 뚜껑 잠금 ✅ · 통 무게 {_wkg:.2f}kg'
                                + (f' ±{_wsd:.3f}' if _wsd is not None else '')
                                if _wkg is not None else ' · 🧢 뚜껑 잠금 ✅ (무게 미측정)')
                        else:
                            batch_outcome += (f' · ⚠️ 뚜껑 닫기 실패: '
                                              f"{(_lid2 or {}).get('error', (_lid2 or {}).get('status', ''))}")
                        # ★불량이 나왔던 주문이면 뚜껑 닫기·무게측정까지 끝난 뒤
                        #   폐기물 처리 시퀀스를 이어서 실행한다(P5에서 바로 P25로).
                        if (self.batch_defect > 0
                                and (_lid2 or {}).get('status') == 'ok'
                                and not self._cycle_stop):
                            self.cycle_msg = f'🗑 불량 {self.batch_defect}개 — 폐기물 처리 시작'
                            _wst = grip_cycle.run_waste(ctx)
                            batch_outcome += (
                                f' · 🗑 폐기물 처리 ✅({self.batch_defect}개)'
                                if (_wst or {}).get('status') == 'ok'
                                else f" · ⚠️ 폐기물 처리 실패: {(_wst or {}).get('error', '')}")
                    if batch_outcome:            # ★'홈 복귀 완료'에 덮이지 않게 최종 결과로 복원
                        self.cycle_msg = batch_outcome
                except Exception as e:
                    self.cycle_msg = f'⚠️ 사이클 오류: {e}'
                finally:
                    MOTION_ACTIVE.clear()
                    ctx.collision_ready = False   # 다음 배치는 시작 시 다시 설정
                    self.cycle_active = False
                    self._training_mode = False
                    self._training_stop = False
                continue

            if self._motion_stop:
                self._motion_stop = False
                with self._lock:
                    self._goto = None       # 대기 이동 취소
                    self._prog = None       # 프로그램도 중단
                soft_stop()

            # ── 블록코딩 프로그램 실행(move/grip/measure/loop/if/call/home) ──
            with self._lock:
                prog = self._prog; pvel = self._prog_vel
            if prog is not None:
                try:
                    exec_prog(prog, pvel)
                except Exception as e:
                    self._prog_msg = f'오류: {e}'
                with self._lock:
                    self._prog = None       # 완료
                continue

            if self._waste_req:
                self._waste_req = False
                MOTION_ACTIVE.set()
                try:
                    if not prepare_collision_for_motion(cfg.COLLISION['fixed']):
                        raise RuntimeError(f"고정 충돌감도({cfg.COLLISION['fixed']}) 준비 실패")
                    self.cycle_msg = '🗑 폐기물 처리 시작'
                    _wr = grip_cycle.run_waste(ctx)
                    self.cycle_msg = ('🗑 폐기물 처리 완료'
                                      if (_wr or {}).get('status') == 'ok'
                                      else f"⚠️ 폐기물 처리 실패: {(_wr or {}).get('error', '')}")
                except Exception as e:
                    self.cycle_msg = f'⚠️ 폐기물 처리 중단: {e}'
                finally:
                    MOTION_ACTIVE.clear()
                continue

            with self._lock:
                mt = self._movetest; self._movetest = None
            if mt is not None:
                key, label, src, dst, vel = mt
                MOTION_ACTIVE.set()
                try:
                    if not prepare_collision_for_motion(cfg.COLLISION['fixed']):
                        raise RuntimeError(f"고정 충돌감도({cfg.COLLISION['fixed']}) 준비 실패")

                    def _mj(pos, v, tag):
                        cur = motion_preflight()
                        acc = max(cfg.SPEED.get('cycle_min_acc', 10.0), v * cfg.SPEED['acc_ratio'])
                        req = MoveJoint.Request()
                        req.pos = [float(x) for x in pos]
                        req.vel = float(v); req.acc = float(acc)
                        req.time = 0.0; req.radius = 0.0
                        req.mode = 0; req.blend_type = 0; req.sync_type = 1
                        audit_motion('MoveJ', pos, cur, v, acc, tag)
                        r = call(cmj, req, cto=5.0)
                        if r is None or not getattr(r, 'success', False):
                            raise RuntimeError(f'MoveJ 거부 — 알람 {last_alarm()}')
                        wait_joint_done(pos, cur, v)

                    if src:                       # ① 출발점으로(안전속도) — 구간 재현 준비
                        self.cycle_msg = f'▶ {label} · 출발점으로 이동 중'
                        _mj(src, float(cfg.SPEED['manual_max']), 'manual')
                        time.sleep(0.4)
                    # ② 실제 구간 — 설정 속도 그대로(1:1). 여기가 체감 대상.
                    self.cycle_msg = f'▶ {label} · {vel:.0f}°/s 로 이동 중'
                    _mj(dst, vel, 'movetest')
                    self.cycle_msg = f'✅ {label} 재현 완료 ({vel:.0f}°/s)'
                except Exception as e:
                    self.cycle_msg = f'⚠️ 이동 테스트 중단: {e}'
                finally:
                    MOTION_ACTIVE.clear()
                continue

            with self._lock:
                g = self._goto; self._goto = None
            if g is not None:
                kind, pos, vel = g
                MOTION_ACTIVE.set()
                try:
                    if not prepare_collision_for_motion(cfg.COLLISION['fixed']):
                        raise RuntimeError(
                            f"고정 충돌감도({cfg.COLLISION['fixed']}) 준비 실패")
                    cur = motion_preflight()
                    if kind == 'j':
                        if not (pos and len(pos) == 6):
                            raise RuntimeError('MoveJ 절대목표 형식 오류')
                        vel, acc = motion_values(vel, 'manual')
                        req = MoveJoint.Request()
                        req.pos = [float(x) for x in pos]
                        req.vel = vel; req.acc = acc
                        req.time = 0.0; req.radius = 0.0
                        req.mode = 0; req.blend_type = 0; req.sync_type = 1
                        audit_motion('MoveJ', pos, cur, vel, acc, 'manual')
                        res = call(cmj, req, cto=5.0)
                        if res is None or not getattr(res, 'success', False):
                            self.last_motion['status'] = 'rejected'
                            raise RuntimeError(f'MoveJ 거부 — 알람 {last_alarm()}')
                        wait_joint_done(pos, cur, vel)
                    else:
                        vel, acc = motion_values(vel, 'manual')
                        req = MoveLine.Request()
                        req.pos = pos; req.vel = [vel * 2.5, vel]; req.acc = [acc * 2.5, acc]
                        req.time = 0.0; req.radius = 0.0; req.ref = 0
                        req.mode = 1; req.blend_type = 0; req.sync_type = 1
                        audit_motion('MoveL', pos, cur, vel, acc, 'manual')
                        res = call(cml, req, cto=5.0)
                        if res is None or not getattr(res, 'success', False):
                            self.last_motion['status'] = 'rejected'
                            raise RuntimeError(f'MoveL 거부 — 알람 {last_alarm()}')
                        wait_motion_done()
                except Exception as e:
                    self.cycle_msg = f'⚠️ 수동 이동 중단: {e}'
                    if self.last_motion:
                        self.last_motion['status'] = 'error'
                        self.last_motion['error'] = str(e)
                finally:
                    MOTION_ACTIVE.clear()
            else:
                time.sleep(0.03)
        try:
            node.destroy_node()
        except Exception:
            pass
