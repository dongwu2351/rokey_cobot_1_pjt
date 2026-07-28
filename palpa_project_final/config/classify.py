"""공 판별 임계값 · 특징 정의 · classify()."""
# 자동 분리: grip_config.py → config/ (내용 동일, 위치만 이동)

import json as _json, os as _os
import paths as _paths             # 데이터 파일 위치(저장소 동봉 data/)

# ── 공 판별 임계값 (총눌림 comp = 접촉폭 기준(grab의 flow_w5) − 40N정착폭, mm) ─
# ★판정 위치 = '잡는 위치(컨테이너 위, 들어올리기 전)'. 더미 받침 영향이 있으므로
#   아래는 컨테이너 위 추정 시작값 — 반드시 라벨링으로 재확정할 것!
#   (라벨 버튼으로 정상/불량/무압 각 3~5개씩 CSV에 모으면 경계 계산)
# 실측 라벨 27개 기반(2026-07-23): 무압 2.0~3.0 / 유압 3.3~3.8(+이상치4.2) / 불량 3.9~4.6
# 경계 3.8 = 불량 10/10 전부 검출(불량→포장 유출이 최악이므로 불량 쪽에 유리하게).
# 현 데이터 정확도 26/27(96%) — 유압 이상치 4.2 하나만 불량으로 오판(안전한 방향).
# ── 공 종류(크기로 자동 구분) × 강성(눌림량)으로 판정 ──────────────────────────
#   테니스공 ≈ 65~68mm, 야구공 ≈ 72~75mm → 접촉크기(size)로 종류 자동 분리.
#   테니스: 무압/유압/구멍(불량)   야구: 소프트/하드 (불량 없음).
#   ★모든 경계값은 '임계값 자동계산'(라벨 CSV 기반)으로 재확정 — 아래는 시작 추정값.
#   런타임에 ball_thresholds.json 이 있으면 이 값을 덮어씀(load_thresholds).
#   실측 크기: 테니스 ≈65~68 / 야구 ≈72~80 / 잘못 잡힘(오파지) ≈86mm.

CLASSIFY = {
    'type_split': 70.3,   # 접촉크기 < 이 값 → 테니스, ≥ → 야구 (라벨로 재확정)
    # 눌림 상한 — 초과하면 '접촉감지 오류'로 보고 재시도(물리적으로 불가능한 값 차단).
    # 2026-07 그리퍼 교체로 파지력이 커져 같은 공도 더 깊이 눌린다.
    # 실측: 구 그리퍼 구멍공 최대 5.5mm → 신 그리퍼 8.7mm 관측 → 여유 포함 10.0.
    # ★자동계산 대상이 아닌 수동 안전값. 실측 최대를 넘는 값이 나오면 다시 올릴 것.
    'comp_max_sane': 10.0,
    'misgrip_min': 80.0,  # 접촉크기 ≥ 이 값 → 오파지/접촉오류 (실공 최대: 테니스 68.5·야구 77)
    'tennis': {
        'nopress_max': 2.35,  # comp ≤ → 🔵 무압(포장)
        'normal_max':  3.28,  # comp ≤ → 🟢 유압(포장) / 초과 → 🔴 구멍(불량)
    },
    'baseball': {
        'by':          'size', # 하드/소프트 판정 차원: 'comp'(눌림) 또는 'size'(크기) — 자동계산이 더 잘 나뉘는 쪽 선택
        'hard_low':    True,   # True: 경계 이하=하드, False: 경계 초과=하드
        'hard_max':    1.71,    # by='comp'일 때: comp ≤ → 🟡 하드 / 초과 → 🟠 소프트
        'hard_size_max': 73.9, # by='size'일 때: 크기 ≤ → 🟡 하드(작게물림) / 초과 → 🟠 소프트(크게물림)
    },
    # 하위호환(옛 코드가 CLASSIFY['nopress_max'] 참조 시): 테니스값 미러
    'nopress_max': 2.35,
    'normal_max':  3.28,
}


# 판정 특징 후보. w40(=크기−눌림)은 5N 접촉 오차가 소거돼 가장 재현성이 높다.
#   size = w5 − offset (5N 접촉 크기),  comp = w5 − w40 (눌림),  w40 = 40N 너비
#   w5 에 오차 ε가 있으면 size·comp 둘 다 +ε 이지만 w40 은 영향 없음 → 실측 검증됨.
# 자동계산 후보는 '크기'와 '눌림' 두 가지만 쓴다.
#   w40/변형률/크립은 표본이 적을 때 우연히 잘 맞아 뽑혔다가 표본이 늘면 무너졌다
#   (실측: w40이 야구 100%→62%, 테니스 100%→67%로 붕괴). 후보를 늘릴수록 이런
#   '우연한 승자'가 뽑힐 확률만 커지므로(다중비교), 물리적으로 명확한 둘만 남긴다.
FEATURES = ('size', 'comp')
FEATURE_LABELS = {'size': '크기', 'comp': '눌림', 'w40': '크기-눌림',
                  'strain': '변형률(눌림÷크기)', 'creep': '크립'}
# 야구 경계값이 저장되는 키(특징별로 다름)
BB_KEY = {'size': 'hard_size_max', 'comp': 'hard_max', 'w40': 'w40_max',
          'strain': 'strain_max', 'creep': 'creep_max'}


def classify(size, comp, creep=None, w40=None):
    """접촉크기(size)로 공 종류, 특징별 임계값으로 등급 판정.

    feat = {'size', 'comp', 'w40'(=크기−눌림), 'creep'}.
    각 경계(무압/유압, 유압/구멍, 하드/소프트)는 자기에게 유리한 특징(by)을 임계값
    파일에서 각각 고른다. 특징값이 없으면 comp로 대체 → 하위호환.
    return (verdict, ball_type, packable). packable: 불량/모호만 False."""
    if size is None or comp is None:
        return ('⚪ 측정없음', None, False)
    if size >= CLASSIFY.get('misgrip_min', 80.0):
        return ('⚠️ 잘못 잡힘 (재시도)', None, False)
    if comp > CLASSIFY.get('comp_max_sane', 6.0):
        # 물리적으로 불가능한 눌림(실측 최대: 구멍 4.6) = 접촉 플래그 오감지 → 재시도
        return ('⚠️ 접촉감지 오류 (재시도)', None, False)
    if w40 is None:
        w40 = size - comp        # size_offset=0 기준. 호출부가 실제 w40을 주면 그걸 우선.
    # 변형률 = 눌림÷크기. 크기로 정규화한 변형이라 공 개체차에 덜 민감(물리적 근거).
    feat = {'comp': comp, 'creep': creep, 'size': size, 'w40': w40,
            'strain': (comp / size if size else None)}

    def pick(by, fallback='comp'):
        v = feat.get(by)
        return feat[fallback] if v is None else v   # 해당 특징 없으면 comp로 대체

    if size >= CLASSIFY['type_split']:                    # ── 야구공 ──
        b = CLASSIFY['baseball']
        by = b.get('by', 'comp')
        metric = pick(by)
        boundary = b.get(BB_KEY.get(by, 'hard_max'), b.get('hard_max', 3.0))
        is_hard = metric <= boundary if b.get('hard_low', True) else metric > boundary
        return ('🟡 하드 (야구)' if is_hard else '🟠 소프트 (야구)', 'baseball', True)

    t = CLASSIFY['tennis']                                # ── 테니스공(이단 캐스케이드) ──
    # ① 무압 여부 — nopress_by 특징이 경계 이하(low)면 무압
    m1 = pick(t.get('nopress_by', 'comp'))
    low1 = t.get('nopress_low', True)
    is_nopress = (m1 <= t['nopress_max']) if low1 else (m1 > t['nopress_max'])
    if is_nopress:
        return ('🔵 무압 (정상)', 'tennis', True)
    # ② 유압 vs 구멍 — normal_by 특징이 경계 이하(low)면 유압, 아니면 구멍(불량)
    m2 = pick(t.get('normal_by', 'comp'))
    low2 = t.get('normal_low', True)
    is_normal = (m2 <= t['normal_max']) if low2 else (m2 > t['normal_max'])
    if is_normal:
        return ('🟢 유압 (정상)', 'tennis', True)
    return ('🔴 구멍 (불량)', 'tennis', False)


# ── 주문 품목/실측 판정 표준 코드 ─────────────────────────────────────────────
# 프론트엔드는 같은 item.id(baseball/tennis_ball)를 쓰고 상품명으로 세부 등급을
# 구분한다. ROS/HTTP/사이클 사이에서는 아래 영문 코드만 주고받아 한글 표시문구
# 변경이 로봇 경로를 깨뜨리지 않게 한다.


_THRESH_PATH = _paths.THRESHOLD_FILE


def load_thresholds():
    """ball_thresholds.json 이 있으면 CLASSIFY 경계값을 덮어씀(자동계산 결과 적용)."""
    try:
        with open(_THRESH_PATH) as f:
            d = _json.load(f)
    except Exception:
        return False
    for k in ('type_split', 'misgrip_min'):
        if k in d:
            CLASSIFY[k] = float(d[k])
    for t in ('tennis', 'baseball'):
        if isinstance(d.get(t), dict):
            for k, v in d[t].items():
                if isinstance(v, str):
                    CLASSIFY[t][k] = v            # 예: baseball['by']='size'
                elif isinstance(v, bool):
                    CLASSIFY[t][k] = v
                else:
                    try:
                        CLASSIFY[t][k] = float(v)
                    except (TypeError, ValueError):
                        CLASSIFY[t][k] = v
    CLASSIFY['nopress_max'] = CLASSIFY['tennis']['nopress_max']
    CLASSIFY['normal_max'] = CLASSIFY['tennis']['normal_max']
    return True


def save_thresholds(d):
    """자동계산된 경계값을 파일로 저장(다음 실행에도 유지)."""
    with open(_THRESH_PATH, 'w') as f:
        _json.dump(d, f, ensure_ascii=False, indent=2)

# ── 그리퍼 동작 파라미터 (RG2, Modbus 직접 명령 [force×10, width×10, 16]) ────
