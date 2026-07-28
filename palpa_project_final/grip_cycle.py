"""[역할] 시퀀스 퍼사드 — sequences/ 의 로봇 동작 시나리오를
`grip_cycle.run_cycle / run_lid / run_waste` 로 노출한다."""
# ─────────────────────────────────────────────────────────────────────────────
# grip_cycle.py — 시퀀스 퍼사드
#
# 실제 구현은 sequences/ 아래 기능별 모듈에 있고, 여기서는 기존 호출부가 쓰던
# 이름(run_cycle / run_lid / run_waste …)을 그대로 다시 노출한다.
# grip_web 이 `grip_cycle.run_cycle(...)` 형태로 부르므로 참조를 고칠 필요가 없다.
#
#   sequences/common.py  중단·트립 예외, 웨이포인트 탐색 헬퍼
#   sequences/order.py   주문 파지 사이클(진입·파지·판정·라우팅·이송·복귀)
#   sequences/lid.py     뚜껑 열기 / 닫기 + 순응 안착 + 통 무게 검증
#   sequences/waste.py   폐기물 처리(폐기통 파지·비움·원위치)
#
# 실측 교훈(중요 — 지우지 말 것):
#  - movej 응답 미확인 → 트립 후 사이클이 로봇 없이 혼자 진행했음
#    → 거부/보호정지는 자동복구·재전송 없이 즉시 중단
#  - '정지'만으로 완료판정 → 감속 크리프에서 조기완료 → 이동 중 다음 명령 = 명령충돌
#    급정지('쿵')+노란불 → '목표도달+정지' 동시충족 + check_motion 확인
#  - 공 쥔 채(40N) 빠른 감속 → 토크 스파이크가 펜던트 안전한계 침범('쿵') → carry 저속
#  - 실기 제어기는 충돌감도를 MANUAL에서만 변경할 수 있다(자동모드=5.7170).
#    배치 시작 전 고정값을 1회 설정하고 이동 중에는 변경하지 않는다.
#  - amovej 는 radius 를 전달하지 않는다 → 코너는 lead 각도(다음 명령 투입 시점)로 제어
#  - 스플라인 실행 중 amovej 를 겹쳐 보내면 컨트롤러가 정지(알람 없는 흰불)
# ─────────────────────────────────────────────────────────────────────────────

from sequences.common import (          # noqa: F401
    CycleStop, TripError, find_wp, find_order_wp, _find_named, _find_named_wp,
)
from sequences.order import run_cycle   # noqa: F401
from sequences.lid import run_lid       # noqa: F401
from sequences.waste import run_waste   # noqa: F401
