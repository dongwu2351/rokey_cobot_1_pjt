"""시퀀스 공통 — 중단/트립 예외, 웨이포인트 탐색 헬퍼."""
# 자동 분리: grip_cycle.py → sequences/ (내용 동일, 위치만 이동)

import grip_config as cfg

class CycleStop(Exception):
    """사용자 중단(■) 요청"""


class TripError(Exception):
    """이동 중 안전정지(외력 등) 감지 — 자동복구·재이동 금지"""


def find_wp(waypoints, role):
    """role 필드 우선, 없으면 이름 키워드 fallback (이름 바꿔도 role 있으면 안 깨짐)"""
    for w in waypoints:
        if w.get('role') == role and w.get('posj'):
            return list(w['posj'])
    for w in waypoints:
        if any(k in w.get('name', '') for k in cfg.WP_ROLE_KEYWORDS.get(role, [])):
            return list(w['posj'])
    return None


def find_order_wp(waypoints, key):
    """주문 사이클용 포인트를 명시적 role 또는 티칭 이름으로 찾는다."""
    role = f'order_{key}'
    for w in waypoints:
        if w.get('role') == role and w.get('posj'):
            return list(w['posj'])
    names = cfg.ORDER_WP_NAMES.get(key, ())
    for wanted in names:
        compact_wanted = ''.join(str(wanted).split())
        for w in waypoints:
            compact_name = ''.join(str(w.get('name', '')).split())
            if compact_wanted in compact_name and w.get('posj'):
                return list(w['posj'])
    return None

def _find_named(waypoints, names):
    for wanted in names:
        cw = ''.join(str(wanted).split())
        for w in waypoints:
            if cw in ''.join(str(w.get('name', '')).split()) and w.get('posj'):
                return list(w['posj'])
    return None


def _find_named_wp(waypoints, names):
    """posj+posx 전체 웨이포인트 dict 반환(순응 구간은 posx 필요)."""
    for wanted in names:
        cw = ''.join(str(wanted).split())
        for w in waypoints:
            if cw in ''.join(str(w.get('name', '')).split()) and w.get('posj'):
                return dict(w)
    return None
