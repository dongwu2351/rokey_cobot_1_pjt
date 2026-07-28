"""JointSub · RobotMonitor — 관절/상태/힘 구독 스레드."""
# 자동 분리: grip_web.py → core/ (내용 동일, 위치만 이동)

import math
import threading
import time

import grip_config as cfg
from core import runtime as RT
from core.rg2 import G
from core.runtime import MOTION_ACTIVE, RCLPY_LOCK, STATE_NAMES


class JointSub(threading.Thread):
    """관절각 실시간 구독 전용 스레드(자체 노드, 구독만 spin).
    /dsr01/joint_states 토픽에서 posj 를 받는다 → 컨트롤러 서비스 부하 0, 실시간.
    (서비스 호출과 섞지 않으므로 rclpy wait-set 크래시 없음)"""
    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.posj = None
        self.last = 0.0
        self._stop = False

    def snapshot(self):
        with self.lock:
            return self.posj, self.last

    def stop(self):
        self._stop = True

    def run(self):
        try:
            import rclpy
            from sensor_msgs.msg import JointState
            from rclpy.executors import SingleThreadedExecutor
        except Exception:
            return
        with RCLPY_LOCK:
            if not rclpy.ok():
                rclpy.init()
        node = rclpy.create_node('grip_web_joints', namespace='dsr01')

        def cb(msg):
            try:
                idx = {n: k for k, n in enumerate(msg.name)}
                js = []
                for j in range(1, 7):
                    nm = 'joint_%d' % j
                    if nm in idx:
                        js.append(math.degrees(msg.position[idx[nm]]))
                if len(js) == 6:
                    with self.lock:
                        self.posj = js
                        self.last = time.time()
            except Exception:
                pass
        node.create_subscription(JointState, 'joint_states', cb, 10)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        while not self._stop and rclpy.ok():
            ex.spin_once(timeout_sec=0.2)
        try:
            node.destroy_node()
        except Exception:
            pass


class RobotMonitor(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.data = {'ros': False, 'connected': False, 'state': None, 'state_name': '-',
                     'servo_on': None,
                     'weight': None, 'weight_est': None, 'weight_total': None, 'tool_force': None,
                     'ext_torque': None, 'joint_torque': None, 'baseline': False,
                     'posj': None, 'posx': None}
        self._reset_req = False
        self._servo_on_req = False
        self._servo_off_req = False
        self._recover_req = False
        self.tool_hist = []           # 공구/TCP 변화 기록
        self._tool_last = (None, None)
        self._tool_ts = 0.0
        self._tool_sample_ts = 0.0
        # 시작할 때 펜던트의 속도를 100%로 덮어쓰지 않는다.
        self._opspeed_req = None
        self._compliance_req = None   # 순응모드: True(켜기)/False(끄기)/None
        self._compliance_stiff = 150.0  # 순응 강성(N/m). 낮을수록 잘 밀림
        self._capture_baseline = False
        self._baseline_fz = None
        self._tf = None               # 마지막(평활화된) 툴 힘 6축 — 무게/힘 안정 표시용
        self._fzbuf = []              # Fz 이동평균 버퍼
        self._js_time = 0.0
        self._stop = False

    def request_reset(self):
        self._reset_req = True

    def request_servo_on(self):
        self._servo_on_req = True
        with self.lock:
            self.data['servo_on'] = True

    def request_servo_off(self):
        self._servo_off_req = True
        with self.lock:
            self.data['servo_on'] = False

    def request_recover(self):
        self._recover_req = True

    def request_opspeed(self, pct):
        self._opspeed_req = max(1, min(cfg.MAX_OPERATION_SPEED, int(pct)))

    def request_compliance(self, on, stiff=None):
        if stiff is not None:
            self._compliance_stiff = max(30.0, min(2000.0, float(stiff)))
        self._compliance_req = bool(on)

    def stop(self):
        self._stop = True

    def snapshot(self):
        with self.lock:
            return dict(self.data)

    def tool_snapshot(self):
        with self.lock:
            return self._tool_last[0], self._tool_last[1], self._tool_sample_ts

    def run(self):
        try:
            import rclpy
            from dsr_msgs2.srv import (GetWorkpieceWeight, GetToolForce, ResetWorkpieceWeight,
                                       GetExternalTorque, GetJointTorque, GetRobotState,
                                       SetRobotMode, SetRobotControl, ServoOff,
                                       GetCurrentPosx, GetCurrentPosj,
                                       TaskComplianceCtrl, ReleaseComplianceCtrl,
                                       SetSafetyMode, ChangeOperationSpeed,
                                       GetCurrentTool, GetCurrentTcp)
        except Exception:
            with self.lock:
                self.data['ros'] = False
            return
        try:
            with RCLPY_LOCK:
                if not rclpy.ok():
                    rclpy.init()
            node = rclpy.create_node('grip_web_monitor', namespace='dsr01')
            cw = node.create_client(GetWorkpieceWeight, 'force/get_workpiece_weight')
            cf = node.create_client(GetToolForce, 'aux_control/get_tool_force')
            cr = node.create_client(ResetWorkpieceWeight, 'force/reset_workpiece_weight')
            cext = node.create_client(GetExternalTorque, 'aux_control/get_external_torque')
            cjt = node.create_client(GetJointTorque, 'aux_control/get_joint_torque')
            cstate = node.create_client(GetRobotState, 'system/get_robot_state')
            cmode = node.create_client(SetRobotMode, 'system/set_robot_mode')
            cctrl = node.create_client(SetRobotControl, 'system/set_robot_control')
            coff = node.create_client(ServoOff, 'system/servo_off')
            cpx = node.create_client(GetCurrentPosx, 'aux_control/get_current_posx')
            cpj = node.create_client(GetCurrentPosj, 'aux_control/get_current_posj')
            cops = node.create_client(ChangeOperationSpeed, 'motion/change_operation_speed')
            cgt = node.create_client(GetCurrentTool, 'tool/get_current_tool')
            cgtcp = node.create_client(GetCurrentTcp, 'tcp/get_current_tcp')
            ccomp_on = node.create_client(TaskComplianceCtrl, 'force/task_compliance_ctrl')
            ccomp_off = node.create_client(ReleaseComplianceCtrl, 'force/release_compliance_ctrl')
            # 전용 executor (스레드마다 분리 → global executor 공유로 인한 wait-set 크래시 방지)
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor()
            executor.add_node(node)
        except Exception:
            with self.lock:
                self.data['ros'] = True
            return

        def call(cli, req, rto=0.12, cto=0.6):
            try:
                if not cli.wait_for_service(timeout_sec=rto):
                    return None
                fut = cli.call_async(req)
                executor.spin_until_future_complete(fut, timeout_sec=cto)
                return fut.result()
            except Exception:
                return None

        i = 0
        while not self._stop and rclpy.ok():
            # 서보 OFF는 긴급 요청이므로 이동 중에도 우선 처리한다.
            if self._servo_off_req:
                self._servo_off_req = False
                o = ServoOff.Request(); o.stop_type = 2
                rr = call(coff, o)
                if rr is None or not getattr(rr, 'success', False):
                    with self.lock:
                        self.data['servo_on'] = None

            # 자동 사이클/수동 모션 중에는 속도·충돌감도·복구·순응 명령과 모니터
            # 서비스가 끼어들지 않게 모두 보류한다.
            cycle_busy = bool(RT.JOG and RT.JOG.cycle_active)
            if MOTION_ACTIVE.is_set() or cycle_busy:
                time.sleep(0.05)
                continue

            # ── 대기 중 명령 처리(격리) ──
            if self._servo_on_req:
                self._servo_on_req = False
                m = SetRobotMode.Request(); m.robot_mode = 1
                rm = call(cmode, m)
                c = SetRobotControl.Request(); c.robot_control = 3
                rc = call(cctrl, c)
                if (rm is None or not getattr(rm, 'success', False)
                        or rc is None or not getattr(rc, 'success', False)):
                    with self.lock:
                        self.data['servo_on'] = None
            if self._recover_req:      # 펜던트 "안전복구"와 동일한 상태인지형 복구
                self._recover_req = False
                sres = call(cstate, GetRobotState.Request(), cto=1.0)
                stt = getattr(sres, 'robot_state', -1) if sres else -1
                seq = {
                    1: (),
                    5: (2,),       # SAFE_STOP
                    3: (3,),       # SAFE_OFF
                    9: (4, 7),     # SAFE_STOP2
                    10: (5, 7),    # SAFE_OFF2
                    8: (7,),       # RECOVERY
                }.get(stt)
                ok = seq is not None
                for code in (seq or ()):
                    rr = call(cctrl, SetRobotControl.Request(robot_control=code), cto=2.0)
                    if rr is None or not getattr(rr, 'success', False):
                        ok = False
                        break
                    time.sleep(0.4)
                if ok:
                    call(cmode, SetRobotMode.Request(robot_mode=1))
                # E-STOP(6), NOT_READY(15), MOVING/unknown은 자동 사다리 금지.
            if self._reset_req:
                self._reset_req = False
                call(cr, ResetWorkpieceWeight.Request())
                self._capture_baseline = True
            if self._opspeed_req is not None:
                v = self._opspeed_req; self._opspeed_req = None
                op = ChangeOperationSpeed.Request(); op.speed = v
                call(cops, op, cto=1.0)
            if self._compliance_req is not None:
                on = self._compliance_req; self._compliance_req = None
                if on:   # 순응 모드 ON: 낮은 강성 스프링 → 밀면 부드럽게 밀리고 놓으면 복귀
                    k = self._compliance_stiff
                    rq = TaskComplianceCtrl.Request()
                    rq.stx = [k, k, k, k / 8.0, k / 8.0, k / 8.0]
                    rq.ref = 0; rq.time = 0.5
                    call(ccomp_on, rq, cto=0.6)
                else:    # 순응 모드 OFF: 강체 복귀
                    call(ccomp_off, ReleaseComplianceCtrl.Request(), cto=0.6)

            # ⚠️ 컨트롤러 포화 방지가 핵심 — 서비스 호출을 최소화한다.
            #  - get_tool_force 를 30Hz로 부르면 컨트롤러가 포화돼 복구 서비스 타임아웃 +
            #    movej 거부(success=False)가 발생한다. → posj만 매 루프, 나머지는 라운드로빈.
            #  - get_robot_state / get_workpiece_weight 는 호출 금지(모션/서비스 마비).
            got = False
            state = tforce = posj = None
            weight = etorque = jtorque = posx = None
            slot = i % 4                                 # 한 번에 서비스 하나만(포화 방지)
            if slot == 0:
                res = call(cf, self._toolforce_req(GetToolForce))
                if res is not None:
                    got = True
                    if getattr(res, 'success', False):
                        raw = [float(x) for x in res.tool_force]
                        # Fz 이동평균(최근 5개)으로 평활화 → 무게 안정
                        self._fzbuf.append(raw[2])
                        if len(self._fzbuf) > 5:
                            self._fzbuf.pop(0)
                        raw[2] = sum(self._fzbuf) / len(self._fzbuf)
                        self._tf = raw
                        tforce = raw
            elif slot == 1:
                res = call(cext, GetExternalTorque.Request())
                if res is not None and getattr(res, 'success', True):
                    etorque = [float(x) for x in res.ext_torque]
            elif slot == 2:
                res = call(cjt, GetJointTorque.Request())
                if res is not None and getattr(res, 'success', True):
                    jtorque = [float(x) for x in res.jts]
            else:
                pxreq = GetCurrentPosx.Request(); pxreq.ref = 0
                res = call(cpx, pxreq)
                if res is not None and getattr(res, 'success', True):
                    try:
                        posx = [float(x) for x in list(res.task_pos_info[0].data)[:6]]
                    except Exception:
                        posx = None
            i += 1

            # ── 공구/TCP 감시(3초 주기): 기록만 한다. 자동 변경은 payload를 더 위험하게
            # 만들 수 있으므로 시작 전 preflight가 정확한 이름을 요구한다. ──
            if time.time() - self._tool_ts > 3.0:
                self._tool_ts = time.time()
                old_tool, old_tcp = self._tool_last
                tl, tc = old_tool, old_tcp
                tool_ok = tcp_ok = False
                rt = call(cgt, GetCurrentTool.Request(), cto=0.6)
                if rt is not None and getattr(rt, 'success', False):
                    tl = rt.info; tool_ok = True
                rc2 = call(cgtcp, GetCurrentTcp.Request(), cto=0.6)
                if rc2 is not None and getattr(rc2, 'success', False):
                    tc = rc2.info; tcp_ok = True
                sample_ts = time.time() if tool_ok and tcp_ok else None
                if (tool_ok or tcp_ok) and (tl, tc) != self._tool_last:
                    warning = ''
                    if tl != cfg.REQUIRED_TOOL or tc != cfg.REQUIRED_TCP:
                        warning = 'MISMATCH'
                    cyc = RT.JOG.cycle_msg if RT.JOG else ''
                    self.tool_hist.append({'t': time.strftime('%H:%M:%S'),
                                           'tool': tl, 'tcp': tc, 'cycle': cyc,
                                           'warning': warning})
                    self.tool_hist = self.tool_hist[-20:]
                if tool_ok or tcp_ok:
                    with self.lock:
                        self._tool_last = (tl, tc)
                        if sample_ts is not None:
                            self._tool_sample_ts = sample_ts

            # 영점: 평활화된 현재 Fz를 기준으로
            if self._capture_baseline and self._tf is not None:
                self._baseline_fz = self._tf[2]; self._capture_baseline = False
            # 무게는 '유지된 힘값'으로 매 주기 계산 → 깜빡임 없음
            weight_est = weight_total = None
            if self._tf is not None:
                weight_total = round(abs(self._tf[2]) / G, 3)          # 매달린 총무게(그리퍼+물체)
                if self._baseline_fz is not None:
                    weight_est = round(abs(self._tf[2] - self._baseline_fz) / G, 3)  # 물체만

            with self.lock:
                d = self.data
                d['ros'] = True
                d['connected'] = got
                d['state_name'] = '연결됨' if got else '미연결'
                if posj is not None: d['posj'] = posj
                if self._tf is not None: d['tool_force'] = self._tf   # 항상 유지값(평활화)
                if etorque is not None: d['ext_torque'] = etorque
                if jtorque is not None: d['joint_torque'] = jtorque
                if posx is not None: d['posx'] = posx
                if weight_total is not None: d['weight_total'] = weight_total
                if weight_est is not None: d['weight_est'] = weight_est  # None이면 유지(깜빡임 방지)
                d['baseline'] = self._baseline_fz is not None
            time.sleep(0.5)   # 컨트롤러 포화/RT타이밍 방해 방지: 폴링 주기 대폭↓ (서비스당 ~0.5Hz)

    @staticmethod
    def _toolforce_req(GetToolForce):
        req = GetToolForce.Request()
        req.ref = 0  # DR_BASE
        return req
