#!/usr/bin/env python3
"""
exception_handler_node
------------------------
- /robot/status (RobotStatus) 구독
- state == ERROR / ESTOPPED 인 경우 처리:
    - 로그 남기기
    - /inspection/result 로 실패 결과 publish (해당 item이 있는 경우, 백엔드 상태를 무한 RECEIVED로
      남겨두지 않기 위함)
    - TODO: 필요 시 알림(n8n webhook, Slack 등) 연동

TODO(담당자 미정 - 로봇팀과 협의 필요):
- ESTOP 발생 시 main_controller_node에게 "재시도 안 함" 신호를 어떻게 줄지
  (현재는 실패 결과만 publish, 재시도 로직 없음)
- 동일 item에 대해 중복 실패 처리 방지 (현재 item_id 기준 dedup 없음, 필요하면 추가)
- current_order_id는 robot_controller_node가 goal.order_id를 그대로 넣어 publish함 (RobotStatus.msg 참고)
"""

from datetime import datetime, timezone

import rclpy
from rclpy.node import Node

from palpa_interfaces.msg import RobotStatus, InspectionResult


class ExceptionHandlerNode(Node):
    def __init__(self):
        super().__init__('exception_handler_node')

        self.status_sub = self.create_subscription(
            RobotStatus, '/robot/status', self.on_status, 10
        )
        self.result_pub = self.create_publisher(InspectionResult, '/inspection/result', 10)

        self.get_logger().info('exception_handler_node started, watching /robot/status ...')

    def on_status(self, msg: RobotStatus):
        if msg.state not in ('ERROR', 'ESTOPPED'):
            return

        self.get_logger().error(
            f'[{msg.node_name}] state={msg.state} item={msg.current_item_id} msg={msg.message}'
        )

        if not msg.current_item_id:
            # 처리 중인 item이 없는 상태의 에러(예: 초기화 실패)라면 결과 publish 생략
            return

        # 해당 item을 실패 처리하여 백엔드가 무한정 RECEIVED로 남지 않도록 함
        result = InspectionResult()
        result.order_id = msg.current_order_id
        result.item_id = msg.current_item_id
        result.success = False
        result.final_stage = 'REJECTING'
        result.reject_reason = f'robot_exception:{msg.state}:{msg.message}'
        result.stamp = datetime.now(timezone.utc).isoformat()

        self.result_pub.publish(result)


def main():
    rclpy.init()
    node = ExceptionHandlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
