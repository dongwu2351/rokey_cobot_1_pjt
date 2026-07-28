"""속도 프로파일 · 이동별 속도 · 블렌딩 · 충돌감도 · 모션 완료판정."""
# 자동 분리: grip_config.py → config/ (내용 동일, 위치만 이동)

from .classify import CLASSIFY
import json as _json, os as _os
import paths as _paths             # 데이터 파일 위치(저장소 동봉 data/)

# 홈 자세 (관절각 deg) — 사이클 시작/종료 기준점
HOME_POSJ = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 실기 시작 전 반드시 컨트롤러에서 확인할 공구/TCP. 이름이 다르면 payload와
# 무게중심 보상이 달라져 정상 MoveJ 감속 토크가 충돌로 오인될 수 있다.
REQUIRED_TOOL = 'Tool Weight1'
REQUIRED_TCP = 'GripperDA_v1'
TOOL_SAMPLE_MAX_AGE_S = 10.0
JOINT_SAMPLE_MAX_AGE_S = 1.0
START_POSE_TOL_DEG = 5.0
DEFAULT_WORK_VEL = 40.0
DEFAULT_OPERATION_SPEED = 100
MAX_OPERATION_SPEED = 100

# ── 구간별 속도 프로파일 (작업속도 슬라이더 값에 대한 배율) ──────────────────
# 실측 교훈: 공 들고(40N) 빠르게 감속하면 토크 스파이크가 펜던트 안전 힘/토크
# 한계를 쳐서 '쿵'+노란불 → 공 쥔 구간은 배율을 낮게 유지할 것.
# ★설계 원칙: "빠른 순항 + 부드러운 감속" — 트립(쿵/노란불)의 원인은 속도가 아니라
#   '감속률'. 속도는 올리되 가속(=감속) 절대상한으로 스파이크를 막는다.

SPEED = {
    'free':     1.5,   # 빈손 이동 — 기본 40에서 30deg/s 상한
    'approach': 1.0,   # 경유지→잡는위치 진입 — 빠르게(사용자 지정), 감속만 부드럽게
    'extract':  1.0,   # 공 빼내기 — 안 줄이고 작업속도 그대로(빠르게)
    'carry':    0.8,   # 판정 후 공 이송(P3까지) — 빠르게
    'drop':     0.5,   # ★놓는 위치 '도착' 이동 — 포장위치는 팔 뻗는 큰 이동이라
                       #   40에선 도착 감속 스파이크로 쿵+노란불(실측, 불량위치는 OK).
                       #   라벨링 수십회 무사고였던 20으로 이 구간만 제한.
    'acc_ratio':      1.0,   # 가속도 = 속도 × 1.0 (아래 절대상한으로만 제한)
    'acc_ratio_carry': 1.0,  # 공 든 구간도 동일 — 상한으로만 제어
    'min_vel':  5.0,
    'min_acc':  5.0,
    'cycle_min_acc': 10.0,  # 자동 주문 이동만 기존 대비 약 2배 가감속
    'max_vel':  100.0,
    # 절대 '속도' 상한(deg/s)
    # 실제 로봇 재검증 전 보수적 상한. UI 값을 올려도 각 구간은 아래 값을 넘지 않는다.
    'free_max':     30.0,
    'approach_max': 24.0,
    'extract_max':  20.0,
    'carry_max':    16.0,
    'drop_max':     12.0,
    'drop_acc_max':      60.0,
    # 절대 '가속(감속)' 상한(deg/s²) — ★스파이크 제거의 핵심(속도 올려도 감속은 부드럽게)
    # 감속상한: 과도하게 낮추면(12~30) 짧은 이동이 전부 느려짐(가속구간 지배) — 실측 복원.
    #   트립의 진짜 원인은 '충돌0 설정 누락 사이클'이었음(아래 col 검증으로 해결).
    'free_acc_max':      60.0,
    'approach_acc_max':  60.0,
    'extract_acc_max':   60.0,
    'carry_acc_max':     60.0,
                               #   미모델 질량 관성이 커져 안전한계 여유 감소 → 완충
    'manual_max':       12.0,
    'manual_acc_ratio':  0.4,
    'manual_acc_max':   12.0,
    'program_max':      10.0,
    'program_acc_max':  10.0,
}

# ── 충돌 민감도 ─────────────────────────────────────────────────────────────
# 실기 M0609 제어기는 AUTONOMOUS에서 변경하면 5.7170(INVALID_SYSTEM_STATE)을
# 반환하고 MANUAL에서만 허용한다. 따라서 배치 시작 전 STANDBY/IDLE에서
# MANUAL → 감도 설정 → AUTONOMOUS 복귀를 딱 한 번 수행하며, 이동 구간 중에는
# 값을 바꾸지 않는다. 구간별 동적 변경은 모드 전환 반복과 명령 실패를 유발한다.

COLLISION = {
    'fixed': 10,
}

import json as _json, os as _os


# ── 연속 이송 ───────────────────────────────────────────────────────────────
# 일반 async MoveJ는 radius가 드라이버에서 버려지므로 lead 시점 선행명령은 보조 수단이다.
# 사이클 경계 P5의 진짜 무정지 통과는 제한된 MoveSplineJoint(return_spline)가 담당한다.
# ★적용 대상은 '순수 통과용' 경유점뿐. 충돌 회피용으로 티칭된 경유점
#   (P2/P4 슬롯진입, P13/P15 보충접근, P12→P10 뚜껑안착)에 쓰면 코너를 자르면서
#   회피가 무효가 되어 충돌한다. 현재는 이송 구간(P2/P4→P5→P6/P7)에만 적용.
# 블렌딩 구간은 경유점에서 감속하지 않으므로 코너링 관성이 커진다 → 속도를 낮춰 시작.
BLEND = {
    'enabled':  True,
    'radius':   10.0,   # 하위호환 설정(현재 async MoveJ 요청에는 전달하지 않음)
    # ★다음 명령을 '고정 시간' 뒤에 보내면 안 된다. 긴 이동일수록 훨씬 일찍 꺾여
    #   경유점을 통째로 건너뛴 것처럼 보인다(실측: P12→P5 115°인데 0.25s면 13% 지점에서 전환).
    #   그래서 '남은 관절거리'로 판단한다 — 경유점에 이만큼 가까워지면 다음 명령 투입.
    #   값이 클수록 코너가 크게 잘리고, 작을수록 경유점에 바짝 붙어 돈다.
    'lead_deg':  25.0,   # 일반 경유점(P9·P13/P15 등) 코너
    'waste_lead_deg': 15.0,      # 폐기물 경유점 기본 코너(P25·P27·P29)
    'waste_lead_hub': 25.0,      # P5 · P28(버리러 갈 때) — 여유 있는 구간이라 크게
    # 복귀 P29: 벗어나는 첫 부분에 걸리는 구조물이 있어 코너를 거의 자르지 않는다.
    # 정지시키지 않으면서 티칭 경로를 바짝 따라가게 하는 값.
    'waste_lead_tight': 4.0,
    'lead_deg_slot': 18.0,  # P2/P4 슬롯 위 경유지 코너
    'lead_deg_hub': 45.0,  # P5 출입 코너 — 사방이 트여 크게 돌아도 안전
    # ★사이클 경계 없애기 — 놓기 후 P5로 '복귀해서 멈추는' 대신, 다음 목표의 파지 경유지까지
    #   한 흐름으로 이어붙인다(P16→⊙P15→⊙P5→P2). 사이클마다 P5에서 서 있던 시간이 사라진다.
    'chain_cycles': True,  # ° — P5(전체 경유지)는 사방이 트여 있어 더 크게 돌아도 안전 → 더 부드럽게
    # async MoveJ는 드라이버가 radius를 버리므로 사이클 사이의 진짜 무정지 통과는
    # MoveSplineJoint를 사용한다. 단, 실기에서 문제가 있었던 P20/P3 슬롯 하강은 절대
    # 스플라인에 넣지 않고 P6/P7(또는 P13/P15)→P5→P2/P4까지만 연결한다.
    # 스플라인은 끝점에서 반드시 완전 정지한다 → P2/P4 '살짝 멈춤'의 원인.
    # amovej→amovej 덮어쓰기는 이 로봇에서 무정지 전환이 검증됐고 모션 충돌도 없다
    # (충돌은 '스플라인 실행 중 amovej' 조합에서만 발생). 그래서 순수 amovej 체인 사용.
    'return_spline': False,
    'lead_timeout_s': 30.0,
}

SPEED_SEGMENTS = ['free', 'approach', 'extract', 'carry', 'drop']  # (구)프로파일 — 이동별 기본값 근거
_SPEED_PATH = _paths.SPEED_FILE

# ── 흐름별 '웨이포인트 이동' 세부 속도 (이동 1:1 개별 조절) ────────────────────
# 각 이동은 고유 key로 개별 조절. MOVE_SPEED에 값 있으면 그 값, 없으면 default 사용.
# kind: 'j'=관절 deg/s, 'l'=직선 mm/s(순응 하강/Z), 'r'=순응 회전 deg/s
FLOW_MOVES = [
    # from/to = 실제 이동의 출발·도착 웨이포인트 키(ORDER_WP_NAMES / LID_WP_NAMES).
    #   '속도' 탭의 ▶이동 버튼이 이 구간을 그대로 재현해 속도를 눈으로 확인시킨다.
    #   테니스/야구로 갈리는 구간은 테니스 쪽을 대표로 쓴다(같은 속도값을 공유하므로).
    #   kind 'l'/'r'(순응·직선)은 순응제어가 필요해 단독 재현이 위험 → test=False.
    {'key': 'ord_enter',      'flow': '주문 파지', 'label': 'P5 허브 진입',       'edge': '→P5',          'profile': 'free',     'default': 30, 'kind': 'j', 'from': None,                'to': 'common'},
    {'key': 'ord_to_lift',    'flow': '주문 파지', 'label': '잡는위치 위 경유',    'edge': 'P5→P2',        'profile': 'free',     'default': 30, 'kind': 'j', 'from': 'common',            'to': 'tennis_lift'},
    {'key': 'ord_descend',    'flow': '주문 파지', 'label': '슬롯으로 하강(파지)', 'edge': 'P2→P20',       'profile': 'approach', 'default': 24, 'kind': 'j', 'from': 'tennis_lift',       'to': 'tennis_pick'},
    {'key': 'ord_extract',    'flow': '주문 파지', 'label': '공 빼내기(40N)',     'edge': 'P20→P2',       'profile': 'extract',  'default': 20, 'kind': 'j', 'from': 'tennis_pick',       'to': 'tennis_lift'},
    {'key': 'ord_carry_hub',  'flow': '주문 파지', 'label': '허브로 운반',        'edge': 'P2→P5',        'profile': 'carry',    'default': 16, 'kind': 'j', 'from': 'tennis_lift',       'to': 'common'},
    {'key': 'ord_carry_via',  'flow': '주문 파지', 'label': '보충 경유지로',      'edge': 'P5→P15',       'profile': 'carry',    'default': 16, 'kind': 'j', 'from': 'common',            'to': 'tennis_refill_via'},
    {'key': 'ord_place',      'flow': '주문 파지', 'label': '놓는 위치로 하강',    'edge': 'P5→P6 포장',   'profile': 'drop',     'default': 12, 'kind': 'j', 'from': 'common',            'to': 'pack'},
    {'key': 'ord_return',     'flow': '주문 파지', 'label': 'P5로 복귀',         'edge': 'P6→P5',        'profile': 'free',     'default': 30, 'kind': 'j', 'from': 'pack',              'to': 'common'},
    {'key': 'lidopen_travel', 'flow': '뚜껑 열기', 'label': '뚜껑 일반 이송',     'edge': 'P5→P12',       'profile': 'free',     'default': 12, 'kind': 'j', 'from': 'common',            'to': 'lid_mount_pre'},
    {'key': 'lidopen_near',   'flow': '뚜껑 열기', 'label': '뚜껑 정밀 접근·이탈', 'edge': 'P12→P10',      'profile': 'drop',     'default': 8,  'kind': 'j', 'from': 'lid_mount_pre',     'to': 'lid_mount'},
    {'key': 'lidclose_travel','flow': '뚜껑 닫기', 'label': '뚜껑 일반 이송',     'edge': 'P5→P9',        'profile': 'free',     'default': 12, 'kind': 'j', 'from': 'common',            'to': 'lid_via'},
    {'key': 'lidclose_near',  'flow': '뚜껑 닫기', 'label': '뚜껑 정밀 접근',     'edge': 'P9→P8',        'profile': 'drop',     'default': 10, 'kind': 'j', 'from': 'lid_via',           'to': 'lid_grab'},
    {'key': 'waste_travel',   'flow': '폐기물 처리', 'label': '폐기물 일반 이송',   'edge': 'P5·P25·P28',   'profile': 'free',     'default': 60, 'kind': 'j', 'from': 'common',       'to': 'waste_via'},
    {'key': 'waste_near',     'flow': '폐기물 처리', 'label': '폐기통 접근·이탈',   'edge': 'P25→P26',      'profile': 'approach', 'default': 60, 'kind': 'j', 'from': 'waste_via',    'to': 'waste_pick'},
    {'key': 'waste_carry',    'flow': '폐기물 처리', 'label': '폐기통 운반',       'edge': 'P27→P28→P29',  'profile': 'carry',    'default': 60, 'kind': 'j', 'from': 'waste_lift',   'to': 'waste_drop'},
    {'key': 'waste_dump',     'flow': '폐기물 처리', 'label': '비우기(기울임)',    'edge': 'P29→P30',      'profile': 'drop',     'default': 60, 'kind': 'j', 'from': 'waste_drop',   'to': 'waste_dump'},
    {'key': 'lid_comp_descend','flow': '뚜껑 닫기','label': '순응 하강(안착)',    'edge': 'P12→P10',      'profile': None,       'default': 8,  'kind': 'l', 'from': None, 'to': None, 'test': False},
    {'key': 'lid_comp_rotate','flow': '뚜껑 닫기', 'label': '회전 잠금',         'edge': 'P10→P11',      'profile': None,       'default': 12, 'kind': 'r', 'from': None, 'to': None, 'test': False},
    {'key': 'lid_box_lift',   'flow': '뚜껑 닫기', 'label': '통 들기/내리기(Z)',  'edge': '±Z 100mm',     'profile': None,       'default': 20, 'kind': 'l', 'from': None, 'to': None, 'test': False},
]
FLOW_MOVE_BY_KEY = {m['key']: m for m in FLOW_MOVES}
FLOW_MOVE_KEYS = {m['key'] for m in FLOW_MOVES}
_MOVE_DEFAULT = {m['key']: float(m['default']) for m in FLOW_MOVES}
MOVE_SPEED = {}   # key -> 사용자 지정 속도(있으면 default 대신). speed_profile.json에서 복원.

def move_vel(key=None, profile='free'):
    """이동 key의 실제 속도. 사용자지정(MOVE_SPEED) > 이동 default > 구(舊)프로파일 상한."""
    if key and key in MOVE_SPEED:
        return float(MOVE_SPEED[key])
    if key and key in _MOVE_DEFAULT:
        return _MOVE_DEFAULT[key]
    return float(SPEED.get((profile or 'free') + '_max', SPEED.get('free_max', 30.0)))

def load_speed_profile():
    """speed_profile.json 있으면 구간 속도+이동별 속도+op_speed 복원 → '속도' 탭 설정 유지."""
    try:
        with open(_SPEED_PATH) as f:
            d = _json.load(f)
    except Exception:
        return None
    for seg in SPEED_SEGMENTS:
        if seg in d:
            SPEED[seg + '_max'] = float(d[seg])   # 구간 deg/s = 1:1 실제속도
    for k, v in (d.get('moves') or {}).items():   # 이동별 개별 속도
        if k in FLOW_MOVE_KEYS:
            try:
                MOVE_SPEED[k] = float(v)
            except (TypeError, ValueError):
                pass
    op = d.get('op_speed')
    return int(op) if op is not None else None

def save_speed_profile(op_speed=None):
    d = {seg: float(SPEED.get(seg + '_max', 30.0)) for seg in SPEED_SEGMENTS}
    d['moves'] = {k: float(v) for k, v in MOVE_SPEED.items()}
    if op_speed is not None:
        d['op_speed'] = int(op_speed)
    else:
        try:
            with open(_SPEED_PATH) as f:
                d['op_speed'] = int(_json.load(f).get('op_speed', DEFAULT_OPERATION_SPEED))
        except Exception:
            d['op_speed'] = DEFAULT_OPERATION_SPEED
    with open(_SPEED_PATH, 'w') as f:
        _json.dump(d, f, ensure_ascii=False, indent=1)


MOTION = {
    'near_deg': 1.0,      # 목표 관절오차 허용(deg) — 0.5는 상대이동 누적오차(실측 J5 0.6°)에
                          #   걸려 '도착했는데 미완료 45s' 오탐. 정지+오차잔류 시 재조준으로 보정.
    'correct_n': 2,       # 정지했는데 목표 밖이면 잔여 delta 재전송 횟수
    'poll_s': 0.2,        # 폴링 간격
    'still_deg': 0.05,    # '정지' 판정: poll당 변화량(deg)
    'confirm_n': 2,       # (도달+정지) 연속 (0.4s — idle_confirm 이중가드가 있어 안전)
    'idle_confirm': 2,    # check_motion IDLE 연속 확인(완료레이스 방지의 주 가드)
    'settle_s': 0.15,     # 완료 후 정리 시간
    'timeout_s': 45.0,    # 이동 타임아웃
    'use_check_motion': True,   # check_motion 서비스로 완료 가속(폴링 병행)
    'use_blend': False,   # dsr_controller2 async MoveJ는 radius를 전달하지 않으므로 사용 금지
    'blend_radius': 30.0, # 이송(경유→P3→놓기) 통과 반경
    'blend_radius_entry': 10.0,  # 진입(홈→경유→잡기) 통과 반경 — 컨테이너 근접 구간이라 타이트하게
    'use_spline': False,  # 슬롯 진입(P2/P4→P20/P3) movesj는 금지. 실기에서 P3 걸림 재현됨.
                          # P5 통과 전용 스플라인은 BLEND['return_spline']로 별도 제한한다.
}
