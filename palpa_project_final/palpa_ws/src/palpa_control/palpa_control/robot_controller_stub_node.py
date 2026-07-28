#!/usr/bin/env python3
"""
robot_controller_stub_node
----------------------------
실제 로봇 없이 통합 테스트할 때 robot_controller_node 대신 실행하는 시뮬레이션 노드.
동일한 액션(/palpa/process_item), 동일한 상태 토픽(/robot/status)을 사용하므로
main_controller_node / exception_handler_node / 백엔드 쪽은 코드 변경 없이 그대로 테스트 가능.

랜덤 pass/fail로 PACKING/REJECTING 결과를 냄.
"""

import random
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from palpa_interfaces.action import ProcessItem
from palpa_interfaces.msg import RobotStatus

STAGES = [
    'PICKING',
    'WEIGHING',
    'MEASURING_DIAMETER',
    'COMPRESSING',
    'CLASSIFYING',
]


class RobotControllerStubNode(Node):
    def __init__(self):
        super().__init__('robot_controller_stub_node')
        cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            ProcessItem,
            '/palpa/process_item',
            execute_callback=self.execute_callback,
            callback_group=cb_group,
        )
        self.status_pub = self.create_publisher(RobotStatus, '/robot/status', 10)

        self._publish_status('IDLE', '', 'stub node 시작됨 (시뮬레이션 모드)')
        self.get_logger().info('robot_controller_stub_node started (SIMULATION)')

    def _publish_status(self, state, current_item_id, message, current_order_id=''):
        msg = RobotStatus()
        msg.node_name = 'robot_controller_stub_node'
        msg.state = state
        msg.current_order_id = current_order_id
        msg.current_item_id = current_item_id
        msg.message = message
        self.status_pub.publish(msg)

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        item_id = goal.item_id
        self._publish_status('BUSY', item_id, f'{item_id} 시뮬레이션 처리 시작', current_order_id=goal.order_id)

        for i, stage in enumerate(STAGES):
            feedback = ProcessItem.Feedback()
            feedback.item_id = item_id
            feedback.current_stage = stage
            feedback.progress = (i + 1) / (len(STAGES) + 1)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.3)

        passed = random.random() > 0.15  # 대략 85% 합격률로 시뮬레이션
        final_stage = 'PACKING' if passed else 'REJECTING'

        feedback = ProcessItem.Feedback()
        feedback.item_id = item_id
        feedback.current_stage = final_stage
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        time.sleep(0.2)

        goal_handle.succeed()
        self._publish_status('IDLE', '', f'{item_id} 시뮬레이션 처리 완료')

        result = ProcessItem.Result()
        result.success = passed
        result.item_id = item_id
        result.final_stage = final_stage
        result.measured_weight = round(random.uniform(55.0, 60.0), 2)
        result.measured_diameter = round(random.uniform(64.0, 69.0), 2)
        result.measured_elasticity = round(random.uniform(0.50, 0.60), 3)
        result.reject_reason = '' if passed else 'threshold_out_of_range (simulated)'
        return result


def main():
    rclpy.init()
    node = RobotControllerStubNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
