"""[역할] palpa_control 패키지 — 팀 통합용 ROS2 노드 3종.

  main_controller_node       주문 수신 → 아이템별 작업 지시(액션 goal) → 결과 보고
  robot_controller_node      액션 수행: grip_web 콘솔에 HTTP 위임 → 실제 로봇 동작
  exception_handler_node     로봇 상태 감시 → 이상 시 실패 결과 대신 발행

  robot_controller_stub_node 로봇 없이 통합 테스트할 때 쓰는 시뮬레이션 대역
  order_contract             주문 payload → 로봇 SKU 변환(순수 함수)
"""
