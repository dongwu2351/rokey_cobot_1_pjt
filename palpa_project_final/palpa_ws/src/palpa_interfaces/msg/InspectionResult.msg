# main_controller_node 가 /inspection/result 로 publish
# order_bridge_node(FastAPI 쪽)가 구독해서 DB 상태 업데이트에 사용

string order_id
string item_id
bool success
string final_stage          # PACKING 또는 REJECTING
float32 measured_weight
float32 measured_diameter
float32 measured_elasticity
string reject_reason
string stamp                # ISO8601 문자열, 처리 완료 시각
