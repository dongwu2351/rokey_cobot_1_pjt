#!/usr/bin/env python3
"""
main_controller_node
---------------------
- /order/new (std_msgs/String, JSON) 구독 -> 백엔드(order_bridge_node)로부터 신규 주문 수신
- 묶음(bundle)별 threshold(무게/지름/탄성) 결정 후 ProcessItem action goal 전송 (robot_controller_node로)
- action result 수신 시 /inspection/result 로 publish -> 백엔드가 구독해서 DB 상태 업데이트

★작업 단위 = '묶음(bundle)' 이다 (품목이 아니다)
   프론트/백엔드가 한 주문을 여러 묶음으로 쪼개 보낸다. 묶음 하나가 곧 포장 튜브 하나이고,
   로봇 쪽에서는 grip_web 배치 하나(= 뚜껑 열기 → 공 채우기 → 뚜껑 닫기)와 1:1 대응한다.
     items[].bundle  이 품목이 몇 번째 묶음에 속하는지 (1-based)
     bundleCount     이 주문의 묶음 총 개수
   한 묶음에는 공이 최대 3개 들어가며 종류가 섞일 수 있다(무압 테니스 2 + 하드 야구 1 등).
   두 필드가 없는 구버전 payload 는 '전부 묶음 1' 로 처리해 기존 동작과 같아진다(하위호환).

TODO(main_controller 담당자):
- 아이템 종류별 threshold 값을 설정 파일(yaml) 또는 DB 조회로 분리할지 결정
- 현재는 item_type 문자열 기준으로 하드코딩된 딕셔너리 사용 중, 실제 검사 스펙 확정되면 교체
"""

import json
from collections import defaultdict, deque
from datetime import datetime, timezone

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from palpa_interfaces.action import ProcessItem
from palpa_interfaces.msg import InspectionResult
from .order_contract import canonical_item_type, target_family

# item_type 별 임시 threshold (확정 전 임시값, 로봇팀/QA팀과 합의 후 교체 필요)
ITEM_THRESHOLDS = {
    "tennis_ball": {
        "weight_min": 56.0, "weight_max": 59.4,
        "diameter_min": 65.4, "diameter_max": 68.6,
        "elasticity_min": 0.53, "elasticity_max": 0.58,
    },
    "baseball": {
        "weight_min": 141.7, "weight_max": 148.8,
        "diameter_min": 72.0, "diameter_max": 74.7,
        "elasticity_min": 0.50, "elasticity_max": 0.58,
    },
}


class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        cb_group = ReentrantCallbackGroup()

        self._action_client = ActionClient(
            self, ProcessItem, '/palpa/process_item', callback_group=cb_group
        )

        self.order_sub = self.create_subscription(
            String, '/order/new', self.on_new_order, 10, callback_group=cb_group
        )

        self.result_pub = self.create_publisher(InspectionResult, '/inspection/result', 10)

        # 묶음 1개 = 튜브 1개 = 배치 1개. 묶음 내 총 수량을 goal에 실어 robot_controller가
        # grip_web 배치 모드로 실행한다. 카운팅과 안전 순차이동은 grip_web이 담당한다.
        self._queue = deque()     # 대기 goal [{order_id,bundle,item_type,th,qty}]
        self._active = None       # 현재 진행 중 (로봇은 하나 — 묶음도 직렬 처리)
        self._halted = False      # 한 배치라도 실패하면 작업자 확인 전 다음 배치 자동실행 금지
        self._retry_timer = self.create_timer(2.0, self._pump)
        self.get_logger().info('main_controller_node started, waiting for /order/new ...')

    def on_new_order(self, msg: String):
        try:
            order = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'/order/new 메시지 JSON 파싱 실패: {msg.data}')
            return

        order_id = order.get('orderId', 'UNKNOWN')
        items = order.get('items', [])
        declared_bundles = int(order.get('bundleCount', 1) or 1)   # 없으면 구버전 → 1

        # ★같은 bundle 값을 가진 품목들이 한 튜브에 함께 들어간다(종류 혼합 가능).
        grouped = defaultdict(list)
        for item in items:
            grouped[int(item.get('bundle', 1) or 1)].append(item)

        # ★수량(qty) = '포장 목표 개수'. variant를 네 개 표준 SKU로 먼저 변환한다.
        # 주문 하나를 전부 검증한 후에만 큐에 넣어 일부 묶음만 실행되는 상황을 막는다.
        pending = []
        try:
            if not grouped:
                raise ValueError('items 가 비어 있음')
            if declared_bundles != len(grouped):
                raise ValueError(
                    f'bundleCount({declared_bundles})와 실제 묶음 수({len(grouped)})가 다름')
            for bundle_idx in sorted(grouped):
                totals = {}                      # 이 묶음의 SKU별 수량 = 튜브 하나의 내용물
                for item in grouped[bundle_idx]:
                    qty = int(item.get('qty', 0))
                    if qty < 1:
                        raise ValueError(f'상품 수량 오류: {qty}')
                    item_type = canonical_item_type(
                        item.get('id'), item.get('variant'), item.get('name', ''))
                    totals[item_type] = totals.get(item_type, 0) + qty
                bundle_qty = sum(totals.values())
                # 튜브 하나에 들어가는 공은 최대 3개(종류 무관) — 주문 전체가 아니라 묶음 기준
                if not 1 <= bundle_qty <= 3:
                    raise ValueError(
                        f'묶음 {bundle_idx}의 수량은 1~3개여야 함(수신={bundle_qty})')
                # 대표 SKU = 이 묶음에서 가장 많은 것(동수면 먼저 나온 것). 로봇이 먼저 노릴 목표이며,
                # 나머지 구성은 order_targets 로 넘어가므로 혼합 묶음도 그대로 채워진다.
                lead_sku = max(totals, key=totals.get)
                pending.append({
                    'order_id': order_id,
                    'bundle': bundle_idx,
                    'item_type': lead_sku,
                    'item_name': ' + '.join(f'{s}x{n}' for s, n in totals.items()),
                    'th': ITEM_THRESHOLDS[target_family(lead_sku)],
                    'qty': bundle_qty,
                    # ★'이 묶음의 구성'만 싣는다 — 주문 전체를 실으면 튜브1을 채우다
                    #   튜브2 몫의 공을 담아 내용물이 주문서와 어긋난다.
                    'order_targets': ','.join(f'{s}:{n}' for s, n in totals.items()),
                })
        except (TypeError, ValueError, KeyError) as e:
            self.get_logger().error(f'주문 {order_id} 계약 오류 — 실행하지 않음: {e}')
            return

        self._queue.extend(pending)
        for batch in pending:
            self.get_logger().info(
                f"주문 {order_id} 묶음 {batch['bundle']}/{len(pending)}: "
                f"{batch['item_name']} → 튜브 1개({batch['qty']}개) 큐잉")
        self._pump()

    def _pump(self):
        """진행 중 배치 없으면 다음 묶음 goal 발사 (묶음은 직렬 — 로봇은 하나)."""
        if self._halted or self._active is not None or not self._queue:
            return
        b = self._queue.popleft()
        self._active = b
        self._send_goal(b)

    def _send_goal(self, b):
        # item_id 에 목표수량('#x{qty}')과 묶음번호('#b{n}') 인코딩 — ProcessItem.Goal에
        # 해당 필드가 없어 여기로 전달한다.
        # ★'#b{n}' 은 생략 불가: robot_controller 의 완료 캐시가 주문 단위로만 묶이면
        #   1번 튜브가 끝난 순간 2번 튜브 goal 이 로봇을 안 움직이고 성공 처리된다.
        #   백엔드도 item_id 첫 토큰을 배치 키로 쓰므로 묶음별로 달라야 결과가 분리 저장된다.
        item_id = (f"{b['item_type']}#x{b['qty']}#b{b['bundle']}"
                   f"#all={b.get('order_targets', '')}")
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('robot_controller 액션 서버 응답 없음 — 2초 뒤 같은 배치 재시도')
            self._queue.appendleft(b)
            self._active = None
            return
        g = ProcessItem.Goal()
        g.order_id = b['order_id']; g.item_id = item_id; g.item_type = b['item_type']
        th = b['th']
        g.weight_threshold_min = th['weight_min']; g.weight_threshold_max = th['weight_max']
        g.diameter_threshold_min = th['diameter_min']; g.diameter_threshold_max = th['diameter_max']
        g.elasticity_threshold_min = th['elasticity_min']; g.elasticity_threshold_max = th['elasticity_max']
        self.get_logger().info(
            f"[{b['order_id']}] 묶음 {b['bundle']} ({b['item_name']}) "
            f"튜브 1개 {b['qty']}개 배치 goal 전송")
        sf = self._action_client.send_goal_async(g, feedback_callback=self._on_feedback)
        sf.add_done_callback(lambda f: self._on_goal_response(f, b, item_id))

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{fb.item_id}] stage={fb.current_stage} progress={fb.progress:.2f}'
        )

    def _publish_failure(self, b, item_id, reason):
        msg = InspectionResult()
        msg.order_id = b['order_id']
        msg.item_id = item_id
        msg.success = False
        msg.final_stage = 'REJECTING'
        msg.reject_reason = reason
        msg.stamp = datetime.now(timezone.utc).isoformat()
        self.result_pub.publish(msg)

    def _on_goal_response(self, future, b, item_id):
        order_id = b['order_id']
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'[{order_id}] {item_id} goal 전송 예외: {e}')
            self._publish_failure(b, item_id, f'goal_send_error:{e}')
            self._halted = True
            self._active = None
            return
        if not goal_handle.accepted:
            self.get_logger().error(f'[{order_id}] {item_id} goal 거부됨')
            self._publish_failure(b, item_id, 'goal_rejected')
            self._halted = True
            self._active = None
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_result(f, b, item_id)
        )

    def _on_result(self, future, b, item_id):
        order_id = b['order_id']
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f'[{order_id}] {item_id} result 수신 예외: {e}')
            self._publish_failure(b, item_id, f'result_error:{e}')
            self._halted = True
            self._active = None
            return

        msg = InspectionResult()
        msg.order_id = order_id
        msg.item_id = result.item_id
        msg.success = result.success
        msg.final_stage = result.final_stage
        msg.measured_weight = result.measured_weight
        msg.measured_diameter = result.measured_diameter
        msg.measured_elasticity = result.measured_elasticity
        msg.reject_reason = result.reject_reason
        msg.stamp = datetime.now(timezone.utc).isoformat()

        self.result_pub.publish(msg)
        self.get_logger().info(
            f'[{order_id}] {result.item_id} -> /inspection/result (success={result.success}, {result.final_stage})'
        )
        # 배치 1개(= 묶음 1개 = 튜브 1개) 완료 — grip_web이 포장수 카운팅까지 끝냈으니 다음 묶음으로.
        self._active = None
        if result.success:
            self._pump()
        else:
            self._halted = True
            self.get_logger().error(
                f'[{order_id}] 묶음 실패 — 남은 {len(self._queue)}개 묶음은 '
                '작업자 확인/노드 재시작 전 자동실행하지 않음')


def main():
    rclpy.init()
    node = MainControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
