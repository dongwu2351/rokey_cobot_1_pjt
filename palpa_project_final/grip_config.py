"""[역할] 설정 퍼사드 — config/ 의 모든 설정을 `cfg.SPEED`, `cfg.CLASSIFY` 처럼
한 이름공간으로 노출한다. 코드 전체가 `import grip_config as cfg` 하나만 쓰면 된다."""
# ─────────────────────────────────────────────────────────────────────────────
# grip_config.py — 설정 퍼사드
#
# 실제 정의는 config/ 아래 기능별 모듈에 있고, 여기서는 그것들을 하나의 이름공간
# (cfg.SPEED, cfg.CLASSIFY, cfg.GRIPPER …)으로 다시 노출한다.
# 기존 코드가 전부 `import grip_config as cfg` 로 쓰고 있으므로, 이 퍼사드를 두면
# 참조를 하나도 고치지 않고 파일만 나눌 수 있다.
#
#   config/motion.py     속도 프로파일 · 이동별 속도 · 블렌딩 · 충돌감도 · 모션 완료판정
#   config/classify.py   공 판별 임계값 · 특징 정의 · classify()
#   config/waypoints.py  주문 SKU · 라우팅 · 웨이포인트 이름표/역할
#   config/gripper.py    RG2 그리퍼 동작 파라미터
#
# ★주의: 아래 dict(SPEED/CLASSIFY/GRIPPER/MOVE_SPEED …)는 '같은 객체'를 가리킨다.
#   load_thresholds()/load_speed_profile()이 제자리 갱신만 하므로 퍼사드가 안전하다.
#   새 설정을 추가할 때도 dict 를 통째로 재대입하지 말고 키만 갱신할 것.
# ─────────────────────────────────────────────────────────────────────────────

from config.motion import *      # noqa: F401,F403
from config.classify import *    # noqa: F401,F403
from config.waypoints import *   # noqa: F401,F403
from config.gripper import *     # noqa: F401,F403

# `import *` 는 밑줄로 시작하는 이름을 건너뛰므로 내부 경로/헬퍼를 명시적으로 가져온다.
from config.motion import _SPEED_PATH, _MOVE_DEFAULT      # noqa: F401
from config.classify import _THRESH_PATH                  # noqa: F401
