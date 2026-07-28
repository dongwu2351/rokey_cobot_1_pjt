"""주문 SKU 코드 · 라우팅 · 웨이포인트 이름표 · 역할."""
# 자동 분리: grip_config.py → config/ (내용 동일, 위치만 이동)

ORDER_TARGET_LABELS = {
    'tennis_normal': '테니스 유압',
    'tennis_nopress': '테니스 무압',
    'baseball_hard': '야구 하드',
    'baseball_soft': '야구 소프트',
}

VARIANT_TO_TARGET = {
    'pressurized': 'tennis_normal',
    'pressureless': 'tennis_nopress',
    'hardball': 'baseball_hard',
    'softball': 'baseball_soft',
}


def normalize_order_target(item_id, item_name='', variant=''):
    """주문 id/name 또는 이미 정규화된 값을 사이클 목표 코드로 변환."""
    raw_id = str(item_id or '').strip().lower()
    raw_name = str(item_name or '').strip().lower()
    raw_variant = str(variant or '').strip().lower()
    text = f'{raw_id} {raw_name} {raw_variant}'

    if raw_id in ORDER_TARGET_LABELS:
        return raw_id
    if raw_id == 'tennis_defect':
        return raw_id
    if raw_variant in VARIANT_TO_TARGET:
        return VARIANT_TO_TARGET[raw_variant]
    if raw_id in VARIANT_TO_TARGET:
        return VARIANT_TO_TARGET[raw_id]

    if ('baseball' in text) or ('야구' in text):
        if ('soft' in text) or ('소프트' in text):
            return 'baseball_soft'
        return 'baseball_hard'

    if ('tennis' in text) or ('테니스' in text):
        if ('nopress' in text) or ('무압' in text) or ('딱딱' in text):
            return 'tennis_nopress'
        if ('defect' in text) or ('구멍' in text) or ('불량' in text):
            return 'tennis_defect'
        return 'tennis_normal'
    return None


def classification_code(verdict, ball_type=None):
    """화면용 판정 문자열을 경로 결정용 표준 코드로 변환."""
    text = str(verdict or '')
    if ('잘못' in text) or ('놓침' in text) or ('없음' in text) or ('오류' in text):
        return 'misgrip'
    if ball_type == 'baseball' or '야구' in text:
        if '하드' in text:
            return 'baseball_hard'
        if '소프트' in text:
            return 'baseball_soft'
    if ball_type == 'tennis' or any(x in text for x in ('유압', '무압', '구멍', '불량')):
        if ('구멍' in text) or ('불량' in text):
            return 'tennis_defect'
        if '무압' in text:
            return 'tennis_nopress'
        if ('유압' in text) or ('정상' in text):
            return 'tennis_normal'
    return 'unknown'


def target_ball_type(target):
    target = normalize_order_target(target)
    if target and target.startswith('tennis_'):
        return 'tennis'
    if target and target.startswith('baseball_'):
        return 'baseball'
    return None


def routing_decision(target, observed, remaining=None):
    """목표 등급과 실측 등급으로 물리 경로를 결정.

    pack: 주문품 포장(P6)
    defect: 테니스 불량함(P7)
    replenish_*: 해당 종류 슬롯에 되돌려 슬롯 새로고침(P13~P16)
    retry: 공을 신뢰할 수 없어 현재 슬롯에서 재시도
    """
    target = normalize_order_target(target)
    if observed == 'tennis_defect':
        return 'defect'
    # 다품목 주문에서는 현재 탐색 목표 하나만 보지 않고 주문 전체의 남은 수량을
    # 본다. 예: 유압을 찾는 중 무압을 집었더라도 무압 주문이 남았으면 즉시 포장.
    if remaining is not None:
        try:
            if int(remaining.get(observed, 0)) > 0:
                return 'pack'
        except (AttributeError, TypeError, ValueError):
            pass
    elif target and observed == target:
        return 'pack'
    if str(observed).startswith('tennis_'):
        return 'replenish_tennis'
    if str(observed).startswith('baseball_'):
        return 'replenish_baseball'
    return 'retry'


# 주문 자동 사이클은 번호가 아닌 이름으로 좌표를 찾는다. 포인트를 재정렬해도 안전하다.
ORDER_WP_NAMES = {
    # P20: 2026-07 재티칭한 테니스 파지/측정 위치. 기존 P1은 사용하지 않는다.
    'tennis_pick': ('테니스공 슬롯 잡는 위치 업데이트',),
    'tennis_lift': ('테니스공 잡고난 경유지', '테니스공 잡고 난 경유지'),
    'baseball_pick': ('야구공 슬롯 잡는 위치',),
    'baseball_lift': ('야구공 잡고 난 경유지', '야구공 잡고난 경유지'),
    'common': ('전체 경유지',),
    'pack': ('포장 품목 넣는 위치',),
    'defect': ('불량 품목 넣는 위치',),
    'baseball_refill_via': ('야구공 슬롯 보충 전 경유지',),
    'baseball_refill_drop': ('야구공 슬롯 보충 놓는 위치',),
    'tennis_refill_via': ('테니스공 슬롯 보충 전 경유지',),
    'tennis_refill_drop': ('테니스공 슬롯 보충 놓는 위치',),
}


# ── 뚜껑(리드) 시퀀스 좌표 — 이름으로 조회(주문 사이클과 동일 방식) ─────────────
# 2026-07 재티칭: T8/T10/T11/T12 로 교체. 신규 이름을 앞에 두어 우선 채택하고,
# 구 이름을 뒤에 남겨 신규가 삭제돼도 동작이 끊기지 않게 한다(안전 폴백).
#   T12 Z=245.8 · T10 Z=219.8 → 순응 하강 26mm (구 18mm)
#   T10 C=70.3 → T11 C=-5.0  → 회전 잠금 -39.6° (구 -41.3°)
# ── 폐기물 처리 시퀀스 좌표 (P25~P30) ──────────────────────────────────────
# 불량(구멍)이 나온 주문은 뚜껑 닫기·무게측정까지 끝낸 뒤 이 시퀀스로 폐기통을
# 집어 비우고 제자리에 돌려놓는다.
WASTE_WP_NAMES = {
    'waste_via':    ('폐기물 잡으러 가는 위치 전 경유지',),   # P25
    'waste_pick':   ('폐기물 잡는 위치',),                  # P26
    'waste_lift':   ('폐기물 잡고 난 후',),                 # P27
    'waste_pre':    ('버리기 전 경유지',),                  # P28
    'waste_drop':   ('폐기물 버리는 위치',),                # P29
    'waste_dump':   ('폐기물 처리',),                       # P30
}

LID_WP_NAMES = {
    'lid_grab':         ('T8 업그레이드', '뚜껑잡는위치'),          # P8: 뚜껑 보관/잡는 자리
    'lid_via':          ('뚜껑 잡기 전 경유지',),                  # P9: P8 바로 위 경유
    'lid_mount_pre':    ('T12 업데이트', '뚜껑 장착 직전 위치'),     # P12: 통 위(하강 전)
    'lid_mount':        ('T10 업그레이드', '뚜껑장착위치(회전하기전'), # P10: 안착(잠금 전)
    'lid_mount_locked': ('T11 업그레이드', '뚜껑장착위치(회전 후'),   # P11: 잠금 후
}

# 라벨 → 공 종류 매핑(자동계산·저장용)
LABEL_TYPE = {'유압': 'tennis', '무압': 'tennis', '구멍': 'tennis',
              '소프트': 'baseball', '하드': 'baseball'}
# 종류별 등급 순서(눌림량 오름차순) — 자동계산이 이 순서로 경계를 매김
TENNIS_ORDER = ['무압', '유압', '구멍']    # comp 낮음→높음
BASEBALL_ORDER = ['하드', '소프트']        # 하드(덜눌림)→소프트(많이눌림)

WP_ROLE_KEYWORDS = {
    'pick':   ['잡'],
    'via':    ['경유'],
    'via2':   ['집은'],
    'pack':   ['포장'],
    'defect': ['불량'],
    'home':   ['홈'],
}
WP_ROLES = ['', 'pick', 'via', 'via2', 'pack', 'defect', 'home']  # UI 드롭다운 순서
WP_ROLE_LABELS = {'': '(자동)', 'pick': '잡는위치', 'via': '경유지', 'via2': '집은후경유',
                  'pack': '포장', 'defect': '불량', 'home': '홈'}

# ── 이동 완료 판정 ───────────────────────────────────────────────────────────
