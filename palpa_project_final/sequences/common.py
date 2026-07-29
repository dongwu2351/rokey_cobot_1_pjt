"""시퀀스 공통 — 중단/트립 예외, 외력 자동복구 장치, 웨이포인트 탐색 헬퍼."""
# 자동 분리: grip_cycle.py → sequences/ (내용 동일, 위치만 이동)

import time

import grip_config as cfg

class CycleStop(Exception):
    """사용자 중단(■) 요청"""


class TripError(Exception):
    """이동 중 안전정지(외력 등) 감지.

    외력으로 멈춘 것인지 구분하려면 '어떤 상태로 멈췄는지'가 필요하다.
    state: 5/9=보호정지(외력·충돌), 3/10=서보 꺼짐, 8=복구모드
    alarm: 컨트롤러 알람 코드(예 '5.7060'=충돌 감지)
    """

    def __init__(self, state=None, alarm=None):
        super().__init__(f'안전정지(state={state}, 알람 {alarm})')
        self.state = state
        self.alarm = alarm

    @property
    def is_external_force(self):
        """보호정지/충돌 알람이면 외력으로 본다(서보오프·복구모드는 상태 문제)."""
        return self.state in (5, 9) or str(self.alarm or '').startswith('5.706')


SAFE_STATES = (3, 5, 8, 9, 10)   # 3/10=서보오프 5/9=보호정지(외력) 8=복구모드
DEAD_STATES = (6, 15)            # 비상정지 / NOT_READY — 자동복구 대상 아님


class TripRecovery:
    """외력(보호정지) 트립을 감지·복구하고 '멈춘 지점부터' 이어가게 하는 공용 장치.

    주문·뚜껑·폐기물 세 시퀀스가 각자 복사본을 갖고 있으면 문구와 절차가 어긋나므로
    여기 한 곳에 둔다. 복구 자체는 ctx.recover_trip(=JogWorker.recover_from_trip)이
    수행한다 — 실기에서 검증된 '🛡 로봇 복구' 경로다.

    ★UI(palpa_ui.html)가 접두 이모지로 상태를 구분한다. 문구 형식을 바꾸지 말 것:
        '🛡 외력 감지 — …'   복구 중(앰버)
        '🔧 복구 완료 — …'   재개(초록)
        '🛑 … 자동 복구 중단' 포기(빨강)

    한도(trip_recover_n)는 시퀀스 실행 1회당이다. 같은 자리에서 반복되면 진짜
    장애물일 수 있으므로 무한 재시도하지 않고 작업자를 부른다.
    """

    def __init__(self, ctx, jog, msg, limit=None):
        self.ctx = ctx
        self.jog = jog
        self.msg = msg
        self.total = int(cfg.MOTION.get('trip_recover_n', 2) if limit is None else limit)
        self.left = self.total

    # ── 상태 조회 ────────────────────────────────────────────────────────
    def state(self):
        rr = self.ctx.call(self.ctx.cstate, self.ctx.GetRobotState.Request(), cto=1.0)
        return getattr(rr, 'robot_state', -1) if rr else -1

    def alarm(self):
        la = getattr(self.ctx, 'last_alarm', None)
        try:
            return la() if callable(la) else None
        except Exception:
            return None

    def check(self, st=None):
        """안전정지면 TripError, 복구 불가 상태면 RuntimeError. 정상이면 상태값 반환."""
        st = self.state() if st is None else st
        if st in SAFE_STATES:
            raise TripError(st, self.alarm())
        if st in DEAD_STATES:
            raise RuntimeError(f'복구 금지 로봇 상태({st})')
        return st

    def trip_if_stopped(self):
        """명령이 거부됐을 때: 외력으로 이미 멈춰 거부된 것인지 확인해 TripError로 올린다.

        'MoveJ 거부'는 대개 결과이지 원인이 아니다 — 원인을 외력으로 정확히 보고해야
        자동복구가 걸린다."""
        st = self.state()
        if st in SAFE_STATES:
            raise TripError(st, self.alarm())
        return False

    def soft_stop(self):
        """일반 중단/오류는 감속 정지(DR_STO=2). E-stop을 흉내 내지 않는다."""
        rq = self.ctx.MoveStop.Request()
        try:
            rq.stop_mode = 2
        except Exception:
            pass
        try:
            return self.ctx.call(self.ctx.cstop, rq, cto=2.0)
        except Exception:
            return None

    # ── 복구 ─────────────────────────────────────────────────────────────
    def recover(self, where, exc=None):
        """외력으로 멈춤 → 원인 표시 → '로봇 복구'와 같은 절차로 자동 복구.

        복구에 성공하면 True. 호출부는 '기억하고 있는 절대 목표'로 다시 이동해
        멈춘 지점부터 이어서 진행한다(절대목표라 중복 이동이 없다).
        """
        why = '외력 감지' if (exc is None or getattr(exc, 'is_external_force', True)) else '안전정지'
        al = getattr(exc, 'alarm', None) or self.alarm()
        if self.left <= 0:
            self.msg(f'🛑 {why} 반복({self.total}회) — 자동 복구 중단, 원인 확인 필요')
            return False
        self.left -= 1
        n = self.total - self.left
        self.msg(f'🛡 {why} — 정지 (알람 {al}) · 자동 복구 {n}/{self.total} · {where}')
        self.soft_stop()
        time.sleep(0.4)
        rec = getattr(self.ctx, 'recover_trip', None)
        if not callable(rec) or not rec():
            self.msg(f'⚠️ 자동 복구 실패 — 수동 확인 필요 ({where})')
            return False
        self.msg(f'🔧 복구 완료 — 기억한 위치부터 이어서 진행 · {where}')
        return True

    def guarded(self, fn, where=None):
        """fn 실행 중 외력 트립이 나면 자동 복구 후 '같은 fn'을 다시 실행한다.

        fn 은 절대목표로 움직이는 동작이어야 한다(재실행해도 중복 이동이 없어야 함).
        체인처럼 여러 구간이 있는 동작은 진행 위치를 리스트(at)로 들고 있다가
        복구 후 그 구간부터 이어간다 — 호출부가 그 상태를 소유한다.
        """
        while True:
            try:
                return fn()
            except TripError as exc:
                w = where if where is not None else getattr(self.jog, 'cycle_msg', '')
                if not self.recover(w, exc):
                    raise RuntimeError(
                        '외력 감지 — 자동 복구 한도 초과/실패, 원인 확인 후 재시작') from exc


def trim_passed_steps(steps, cur, near_deg=0.5):
    """체인 시작 시 '이미 그 점 위에 서 있는' 선행 경유점만 잘라낸다.

    전환 체인이 현재 위치에서 시작할 때 같은 자리로 한 번 더 가는 것을 막는 용도다.

    ★위치로 '지나왔는지'를 추정하지 않는다. 관절공간 경유점은 일직선이 아니라서
      '다음 점에 더 가깝다' 같은 판정은 안 지나온 점까지 잘라낸다.
      실측 사고(2026-07-28): 뚜껑 체인 [P5, P9, P24]를 P21에서 시작했더니 그 추정으로
      P5·P9가 통째로 잘려 P24로 직행했다 — 회피 경유가 무효가 되어 충돌 위험.
      외력 복구 후 '어디부터 이어갈지'는 추정이 아니라 **어디까지 명령을 보냈는지**
      (blend_chain 의 at 인덱스)로 정한다.
    """
    if not cur or len(cur) != 6:
        return steps
    while len(steps) > 1:
        here = steps[0][0]
        if max(abs(here[i] - cur[i]) for i in range(6)) < near_deg:
            steps = steps[1:]
            continue
        break
    return steps


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
