# robot_controller_node -> exception_handler_node / main_controller_node 로 publish
# 로봇 현재 상태 및 예외 상황 보고용

string node_name
string state                # IDLE / BUSY / ERROR / ESTOPPED
string current_order_id     # 처리 중인 order, 없으면 빈 문자열
string current_item_id      # 처리 중인 item, 없으면 빈 문자열
string message               # 에러 메시지 등 부가 설명
