#!/usr/bin/env python3
"""
robot_controller_node — 실제 로봇 제어(브릿지 방식)
====================================================
[로봇 제어팀 구현본]

이 노드는 팀 통합 계약(/palpa/process_item 액션, /robot/status 토픽)을 그대로
지키면서, 실제 로봇 동작은 이미 검증된 **grip_web.py(PALPA 오퍼레이터 콘솔)** 에
HTTP로 위임한다. → 드라이버·그리퍼(Modbus)·판정·안전복구 등 실기에서 검증된
스택을 100% 재사용(재테스트 위험 0).

동시 실행 구성:
  1) 로봇 브레인:   cd final_project && python3 -u grip_web.py   (ROS_DOMAIN_ID=60)
  2) 팔 드라이버:   ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real ...
  3) 이 노드:       ros2 run palpa_control robot_controller_node
  4) 팀 노드들:     main_controller_node / exception_handler_node

파이프라인 stage 매핑 (우리 사이클 → 계약 stage):
  홈/경유/개방/진입/잡기 → PICKING
  판정(measure: 5N접촉→40N정착) → WEIGHING → MEASURING_DIAMETER → COMPRESSING → CLASSIFYING
  포장/불량 이송+놓기 → PACKING / REJECTING

판정 주체: 우리 comp 기준(grip_config.CLASSIFY, 실측 라벨링으로 확정)이 그대로 결정.
  - 유압(정상)/무압 → 포장 → success=True,  final_stage=PACKING
  - 불량(구멍/펑크) → 불량함 → success=False, final_stage=REJECTING, reject_reason=defect_hole
  - 놓침/오류        → success=False, REJECTING, reject_reason=grip_failed/robot_error
액션 result의 measured_weight/diameter/elasticity 3값은 실측을 그대로 실어 보고한다.
"""

import json
import threading
import time
import urllib.parse
import urllib.request

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from palpa_interfaces.action import ProcessItem
from palpa_interfaces.msg import RobotStatus

# ── grip_web(로봇 브레인) HTTP 엔드포인트 ──────────────────────────────────────
WEB_BASE = 'http://127.0.0.1:8760'
DEFAULT_VEL = 40          # 자동 주문 MoveJ 약 2배 가속(그리퍼 5N→40N 판별 시간은 불변)
POLL_S = 0.3              # 사이클 진행 폴링 주기
CYCLE_TIMEOUT_S = 180.0   # 판정 시도 1회당 최대 허용시간(배치 상한은 검색 시도수와 연동)


def _parse_qty(item_id):
    """item_id 에 인코딩된 목표 포장수량('type#x3' → 3). 없으면 1."""
    if '#x' in item_id:
        try:
            raw = item_id.split('#x', 1)[1].split('#', 1)[0].split('/', 1)[0]
            return max(1, min(3, int(raw)))
        except ValueError:
            return 1
    return 1


def _parse_bundle(item_id):
    """item_id 에 인코딩된 묶음번호('type#x3#b2' → 2). 없으면 1(구버전 = 묶음 하나)."""
    if '#b' in item_id:
        try:
            return max(1, int(item_id.split('#b', 1)[1].split('#', 1)[0]))
        except ValueError:
            return 1
    return 1


def _parse_order_targets(item_id):
    """'#all=sku:qty,sku:qty'로 전달된 이 묶음(튜브 하나)의 SKU 구성을 복원한다."""
    if '#all=' not in item_id:
        return {}
    spec = item_id.split('#all=', 1)[1].split()[0]
    out = {}
    try:
        for part in spec.split(','):
            sku, raw_qty = part.split(':', 1)
            qty = int(raw_qty)
            if not sku or qty < 1:
                return {}
            out[sku] = out.get(sku, 0) + qty
    except (TypeError, ValueError):
        return {}
    return out

# 우리 사이클 메시지(①~⑪) → 계약 stage 매핑 키워드 (앞선 항목이 우선)
STAGE_BY_KEYWORD = [
    ('포장 위치', 'PACKING'), ('포장위치', 'PACKING'),
    ('불량 위치', 'REJECTING'), ('불량위치', 'REJECTING'),
    ('슬롯 보충', 'REJECTING'),
    ('판정', 'CLASSIFYING'), ('판별', 'CLASSIFYING'),
    ('빼내', 'COMPRESSING'),
    ('잡기', 'PICKING'), ('진입', 'PICKING'),
    ('개방', 'PICKING'), ('경유', 'PICKING'), ('홈', 'PICKING'),
]
STAGE_PROGRESS = {'PICKING': .15, 'WEIGHING': .35, 'MEASURING_DIAMETER': .45,
                  'COMPRESSING': .6, 'CLASSIFYING': .7, 'PACKING': .9, 'REJECTING': .9}


def _http_get(path, timeout=5.0):
    with urllib.request.urlopen(WEB_BASE + path, timeout=timeout) as r:
        return json.loads(r.read().decode('utf-8'))


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        cb = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, ProcessItem, '/palpa/process_item',
            execute_callback=self.execute_callback,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=cb,
        )
        self.status_pub = self.create_publisher(RobotStatus, '/robot/status', 10)
        # ★직렬화 락: main_controller가 여러 아이템 goal을 한꺼번에 쏘면(주문 다건)
        #   파이프라인이 동시 실행돼 같은 사이클을 서로 폴링 → 판정 크로스토크 +
        #   타임아웃 파이프라인이 남의 사이클에 stop_work 발사('그리퍼 오류'로 보임).
        #   로봇은 물리적으로 하나 — 반드시 한 번에 1개만, 나머지는 대기.
        self._pipe_lock = threading.Lock()
        # 한 액션이 그 묶음(튜브 하나)을 통째로 포장한 뒤, main_controller가 보내는
        # 같은 묶음의 후속 액션은 로봇 재동작 없이 성공 응답하기 위한 캐시.
        # ★키는 반드시 (order_id, bundle) — 주문 단위로만 묶으면 1번 튜브가 끝난 순간
        #   2번 튜브 goal 이 캐시에 걸려 로봇이 안 움직이고 성공으로 보고된다.
        self._completed_multi_orders = set()   # {(order_id, bundle), ...}

        ok = self._brain_ok()
        self._publish_status('IDLE', '', '',
                             'robot_controller_node 시작 · grip_web ' + ('연결됨' if ok else '미연결(먼저 실행 필요)'))
        self.get_logger().info(f'robot_controller_node started (bridge -> {WEB_BASE}, brain={"OK" if ok else "DOWN"})')

    def _publish_status(self, state, order_id, item_id, message):
        m = RobotStatus()
        m.node_name = 'robot_controller_node'
        m.state = state
        m.current_order_id = order_id
        m.current_item_id = item_id
        m.message = message
        self.status_pub.publish(m)

    def _brain_ok(self):
        try:
            _http_get('/api/status', timeout=2.0)
            return True
        except Exception:
            return False

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        item_id, order_id = goal.item_id, goal.order_id
        self.get_logger().info(f'goal 수신: order={order_id} item={item_id} type={goal.item_type}')
        self._publish_status('BUSY', order_id, item_id, f'{item_id} 처리 시작')
        try:
            result = self._execute_pipeline(goal, goal_handle)
        except Exception as e:
            try:
                _http_get('/api/stop_work', timeout=2.0)
            except Exception:
                pass
            self.get_logger().error(f'{item_id} 처리 중 예외: {e}')
            self._publish_status('ERROR', order_id, item_id, str(e))
            goal_handle.abort()
            r = ProcessItem.Result()
            r.success = False; r.item_id = item_id
            r.final_stage = 'REJECTING'; r.reject_reason = f'internal_error: {e}'
            return r
        self._publish_status('IDLE', '', '', f'{item_id} 처리 완료 · {result.final_stage}')
        return result

    def _feedback(self, gh, item_id, stage, progress):
        fb = ProcessItem.Feedback()
        fb.item_id = item_id
        fb.current_stage = stage
        fb.progress = float(max(0.0, min(1.0, progress)))
        gh.publish_feedback(fb)

    def _execute_pipeline(self, goal, goal_handle):
        with self._pipe_lock:      # ★한 번에 1 아이템 — 동시 goal은 여기서 줄 서기
            return self._execute_pipeline_locked(goal, goal_handle)

    def _execute_pipeline_locked(self, goal, goal_handle):
        item_id = goal.item_id
        individual_qty = _parse_qty(item_id)
        order_targets = _parse_order_targets(item_id)
        bundle = _parse_bundle(item_id)                    # 묶음 = 튜브 1개 = 배치 1개
        batch_key = (goal.order_id, bundle)
        if order_targets and batch_key in self._completed_multi_orders:
            r = ProcessItem.Result()
            r.success = True
            r.item_id = f'{item_id} 포장{individual_qty}/{individual_qty}'
            r.final_stage = 'PACKING'
            r.reject_reason = ''
            r.measured_weight = 0.0
            r.measured_diameter = 0.0
            r.measured_elasticity = 0.0
            self._feedback(goal_handle, item_id, 'PACKING', 1.0)
            goal_handle.succeed()
            self.get_logger().info(
                f'[{goal.order_id}] 묶음 {bundle} 포장 완료 캐시 → {item_id} 추가 이동 없이 완료')
            return r

        # order_targets 는 '이 묶음의 구성'이므로 합계가 곧 튜브 하나의 목표 수량이다.
        qty = sum(order_targets.values()) if order_targets else individual_qty
        if qty > 3:
            raise RuntimeError(
                f'묶음 {bundle} 목표 {qty}개 — 튜브 용량(3개) 초과. '
                'main_controller 가 묶음 단위로 쪼개지 않았습니다')
        if not self._brain_ok():
            raise RuntimeError('grip_web(로봇 브레인) 미연결 — final_project/grip_web.py 를 먼저 실행하세요')

        # 0) 이전 사이클 잔여가 있으면 완전히 끝날 때까지 대기(겹침 방지)
        for _ in range(100):                       # 최대 30초
            if not _http_get('/api/status').get('cycle', {}).get('active'):
                break
            time.sleep(0.3)
        else:
            raise RuntimeError('이전 grip_web 사이클이 30초 안에 종료되지 않음')

        # 1) ★배치 모드 시작 — 목표수량과 정확한 variant SKU를 함께 전달한다.
        query = urllib.parse.urlencode({
            'count': qty,
            'vel': DEFAULT_VEL,
            'target': goal.item_type,
            **({'targets': ','.join(
                f'{sku}:{n}' for sku, n in order_targets.items())}
               if order_targets else {}),
        })
        start = _http_get('/api/start_work?' + query)
        if not start.get('ok'):
            raise RuntimeError('grip_web 작업 시작 거부: ' + str(start.get('msg', '원인 미상')))
        self._feedback(goal_handle, item_id, 'PICKING', 0.02)

        # 배치가 active 될 때까지 잠깐 대기
        st = {}
        for _ in range(20):
            st = _http_get('/api/status')
            if st.get('cycle', {}).get('active'):
                break
            time.sleep(0.15)
        else:
            msg = st.get('cycle', {}).get('msg', '') if st else ''
            raise RuntimeError('grip_web 사이클 시작 확인 실패' + (f': {msg}' if msg else ''))

        # 2) 배치 전체 진행 폴링 → stage/포장수 feedback, 완료/중단 감지
        t0 = time.time()
        # grip_web은 다른 등급을 슬롯에 돌려보내며 최대 max(20, qty*12)개를
        # 검색한다. 액션 브릿지가 그보다 먼저 끊지 않도록 같은 시도 상한을 반영한다.
        search_attempt_cap = max(20, qty * 12)
        batch_timeout = CYCLE_TIMEOUT_S * search_attempt_cap + 60.0
        last_stage = 'PICKING'
        seen_classify = False
        last_packed = -1
        last_signature = None
        last_progress_at = t0
        while True:
            if time.time() - t0 > batch_timeout:
                _http_get('/api/stop_work')
                raise RuntimeError('batch_timeout')
            if goal_handle.is_cancel_requested:
                _http_get('/api/stop_work')
                goal_handle.canceled()
                r = ProcessItem.Result()
                r.success = False; r.item_id = item_id
                r.final_stage = 'REJECTING'; r.reject_reason = 'canceled'
                return r

            st = _http_get('/api/status')
            cy = st.get('cycle', {})
            msg = cy.get('msg', '') or ''
            packed = int(cy.get('batch_packed') or 0)
            target = int(cy.get('batch_target') or qty)
            signature = (msg, packed, bool(cy.get('active')))
            if signature != last_signature:
                last_signature = signature
                last_progress_at = time.time()
            elif time.time() - last_progress_at > CYCLE_TIMEOUT_S:
                _http_get('/api/stop_work')
                raise RuntimeError('batch_stalled (180초 동안 상태 변화 없음)')

            stage = last_stage
            for kw, sg in STAGE_BY_KEYWORD:
                if kw in msg:
                    stage = sg
                    break
            # 진행률 = (포장완료 + 현재사이클 stage분율) / 목표
            frac = (packed + STAGE_PROGRESS.get(stage, .5)) / max(1, target)
            if stage != last_stage or packed != last_packed:
                self._feedback(goal_handle, item_id, stage, frac)
                last_stage = stage; last_packed = packed
                if stage == 'CLASSIFYING':
                    seen_classify = True

            active = cy.get('active')
            # 배치 종료 신호: 모두 담음 / 공없음 / 연속실패·미개방 중단 (모두 배치 루프가 active=False로 마감)
            ended = ('모두 담' in msg) or ('공이 없습니다' in msg) or ('배치 중단' in msg) or ('사이클 오류' in msg)
            if (not active) and (ended or '완료' in msg):
                break
            time.sleep(POLL_S)

        # 3) 결과 — 배치 요약(포장수 vs 목표)으로 판정
        cy = st.get('cycle', {})
        msg = cy.get('msg', '') or ''
        packed = int(cy.get('batch_packed') or 0)
        target = int(cy.get('batch_target') or qty)
        cls = cy.get('flow_class') or ''       # 마지막 공의 분류(측정값 보고용)
        r = ProcessItem.Result()
        r.item_id = f'{item_id} 포장{packed}/{target}'
        r.measured_weight = float(cy.get('measured_weight') or 0.0)
        r.measured_diameter = float(cy.get('measured_diameter') or 0.0)
        r.measured_elasticity = float(cy.get('measured_elasticity') or 0.0)

        # count만으로 성공 처리하지 않는다. P6 개방 뒤 P5 복귀가 실패해도 count는
        # 이미 커밋되므로, grip_web의 최종 안전 종료 문구까지 확인해야 한다.
        if packed >= target and '모두 담' in msg:
            r.success = True; r.final_stage = 'PACKING'; r.reject_reason = ''
            if order_targets:
                self._completed_multi_orders.add(batch_key)
        elif '공이 없습니다' in msg:
            r.success = False; r.final_stage = 'REJECTING'
            r.reject_reason = f'container_empty (포장 {packed}/{target})'
        else:
            r.success = False; r.final_stage = 'REJECTING'
            r.reject_reason = f'partial (포장 {packed}/{target}): ' + msg[:60]

        if not seen_classify:
            self._feedback(goal_handle, item_id, 'CLASSIFYING', 0.7)
        self._feedback(goal_handle, item_id, r.final_stage, 1.0)

        # 배치가 정상 종료(오류 아님)면 succeed — 공소진으로 덜 담겨도 '정상 처리'로 마감
        if '사이클 오류' not in msg:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info(
            f'{item_id} 배치 결과: 포장 {packed}/{target} (마지막 {cls}) -> {r.final_stage} '
            f'/ {r.reject_reason or "success"}')
        return r


def main():
    rclpy.init()
    node = RobotControllerNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
