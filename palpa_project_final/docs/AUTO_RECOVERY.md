# 외력 자동복구 — 밀면 멈추고, 스스로 풀고, 하던 자리부터 이어간다

> **한 줄 요약**
> 사람이 로봇을 밀면 **보호정지 → 자동 해제 → 멈춘 그 지점부터 재개**까지 사람 손 없이 진행한다.
> 재개 지점은 **추정하지 않고 기억한다** — 어디까지 명령을 보냈는지를 인덱스로 들고 있다.

---

## 목차

1. [무엇을 하는가](#1-무엇을-하는가)
2. [전체 구조](#2-전체-구조)
3. [1단계 — 감지](#3-1단계--감지)
4. [2단계 — 복구](#4-2단계--복구)
5. [3단계 — 재개, 그리고 위치 기억](#5-3단계--재개-그리고-위치-기억)
6. [위치 추정의 함정 (실측 사고)](#6-위치-추정의-함정)
7. [적용 범위 — 작업 전 과정](#7-적용-범위)
8. [자동복구 금지 구역](#8-자동복구-금지-구역)
9. [한도와 포기](#9-한도와-포기)
10. [UI 표시 3단계](#10-ui-표시-3단계)
11. [파라미터 전체](#11-파라미터-전체)
12. [시퀀스 다이어그램](#12-시퀀스-다이어그램)
13. [코드 위치](#13-코드-위치)

---

## 1. 무엇을 하는가

협동로봇은 사람과 같은 공간에서 일하므로 **부딪히면 즉시 멈춰야** 한다(보호정지).
문제는 멈춘 **다음**이다.

**예전 동작:**
```
사람이 팔을 민다 → 보호정지(노란불) → 배치 중단 → 작업자가 와서
  펜던트에서 알람 리셋 → 서보 ON → 대시보드에서 작업 재시작 → 처음부터 다시
```

**지금 동작:**
```
사람이 팔을 민다 → 보호정지 → 자동으로 알람 해제·서보 ON·자율모드 복귀
  → 멈춘 그 이동을 이어서 진행 → 아무 일 없었던 듯 계속
```

작업자 개입 없이 **평균 2~3초** 안에 복귀한다.

---

## 2. 전체 구조

```
┌──────────────────────────────────────────────────────────────┐
│  sequences/  (order · lid · waste)                           │
│                                                              │
│   ┌────────────┐  트립 감지   ┌──────────────┐               │
│   │ 이동 실행   │ ──────────▶ │ TripError    │               │
│   │ (goto/체인) │             └──────┬───────┘               │
│   └─────▲──────┘                    │                        │
│         │                    ┌──────▼─────────┐              │
│         │  재개(at 인덱스)   │ TripRecovery   │              │
│         └────────────────────│  .guarded()    │              │
│                              │  .recover()    │              │
│                              └──────┬─────────┘              │
└─────────────────────────────────────┼────────────────────────┘
                                      │ ctx.recover_trip()
┌─────────────────────────────────────▼────────────────────────┐
│  core/jog.py  recover_from_trip()                            │
│    상태별 복구코드 → RESET_RECOVERY → 서보ON → AUTO → 확인    │
└──────────────────────────────────────────────────────────────┘
```

**역할 분담**

| 계층 | 담당 |
|---|---|
| `sequences/common.py` `TripRecovery` | **감지·판단·재시도** — 언제 트립인지, 몇 번까지 봐줄지, 문구 |
| `core/jog.py` `recover_from_trip()` | **실제 복구** — 로봇 상태를 STANDBY로 되돌리는 DRFL 호출 |
| 각 시퀀스의 `blend_chain`/`goto` | **재개 지점 소유** — `at` 인덱스를 들고 있음 |

이렇게 나눈 이유: 세 시퀀스가 각자 복사본을 가지면 **문구와 절차가 어긋난다.**
실제로 예전엔 `order.py`에만 있었고 뚜껑·폐기물은 그냥 실패했다.

---

## 3. 1단계 — 감지

### 로봇 상태 코드

| 코드 | 이름 | 뜻 | 자동복구 |
|---|---|---|---|
| 1 | `STANDBY` | 정상(서보 ON) | — |
| 2 | `MOVING` | 이동 중 | — |
| **3** | `SAFE_OFF` | 서보 꺼짐 | ✅ |
| **5** | `SAFE_STOP` | **보호정지(외력·충돌)** | ✅ |
| 6 | `EMERGENCY_STOP` | 비상정지 | ❌ 사람이 해제 |
| **8** | `RECOVERY` | 복구모드 | ✅ |
| **9** | `SAFE_STOP2` | 보호정지2 | ✅ |
| **10** | `SAFE_OFF2` | 서보 꺼짐2 | ✅ |
| 15 | `NOT_READY` | 미준비 | ❌ |

```python
SAFE_STATES = (3, 5, 8, 9, 10)   # 자동복구 대상
DEAD_STATES = (6, 15)            # 사람이 해제해야 함
```

### 감지 지점 — 3가지 경로

**① 이동 중 주기 감시 (2초마다)**

```python
if time.time() - last_st > 2.0:
    last_st = time.time()
    trip.check()          # SAFE_STATES 면 TripError
```

> 이 감시가 없으면 외력으로 멈춰도 **도착 타임아웃(45~60초)까지 그냥 서 있는다.**
> 실제로 `lid.py`에 이 감시가 없어서 뚜껑 구간에서 45초를 기다리는 문제가 있었다.

**② 도착 판정 시점**

```python
if okn >= need and motion_idle():
    st = robot_state()
    if st == 1:
        return True
    trip.check(st)        # STANDBY가 아니면 트립
```

**③ 명령이 거부됐을 때 — 원인 재확인**

```python
res = ctx.call(ctx.cmj, req, cto=5.0)
if res is None or not getattr(res, 'success', False):
    trip.trip_if_stopped()          # 외력으로 이미 멈춰서 거절된 건가?
    raise RuntimeError('이동 거부 — 자동복구/재전송 금지')
```

> 📌 **핵심 통찰**: `MoveJ 거부`는 대개 **결과이지 원인이 아니다.**
> 외력으로 이미 보호정지된 로봇이 새 명령을 거절한 것뿐이다.
> 상태를 다시 확인해 **외력이면 외력으로 보고**해야 자동복구가 걸린다.
> 이걸 안 하면 "이동 거부"라는 엉뚱한 에러로 배치가 중단된다.

### 외력인지 상태 문제인지 구분

```python
class TripError(Exception):
    def __init__(self, state=None, alarm=None):
        self.state = state
        self.alarm = alarm

    @property
    def is_external_force(self):
        """보호정지/충돌 알람이면 외력으로 본다(서보오프·복구모드는 상태 문제)."""
        return self.state in (5, 9) or str(self.alarm or '').startswith('5.706')
```

| 판정 | 조건 | UI 문구 |
|---|---|---|
| **외력 감지** | `state ∈ {5, 9}` 또는 알람 `5.706x`(충돌) | `🛡 외력 감지 — …` |
| 안전정지 | 그 외 (서보오프·복구모드) | `🛡 안전정지 — …` |

---

## 4. 2단계 — 복구

`core/jog.py`의 `recover_from_trip()`이 담당한다.
**대시보드의 `🛡 로봇 복구` 버튼과 같은 계열의 절차**로, 실기에서 검증된 순서다.

### 제어 코드

| 코드 | 이름 |
|---|---|
| 2 | `RESET_SAFE_STOP` |
| 3 | `SERVO_ON` |
| 4 | `RECOVERY_SAFE_STOP` |
| 5 | `RECOVERY_SAFE_OFF` |
| 7 | `RESET_RECOVERY` |

### 상태별 복구 시퀀스

```python
st = robot_state()
if st in (5, 9):        # 보호정지(노란불)
    seq = (4, 7, 3)     # RECOVERY_SAFE_STOP → RESET_RECOVERY → SERVO_ON
elif st in (3, 10):     # 서보 꺼짐
    seq = (5, 7, 3)     # RECOVERY_SAFE_OFF → RESET_RECOVERY → SERVO_ON
elif st == 8:           # 이미 복구모드
    seq = (7, 3)        # RESET_RECOVERY → SERVO_ON
else:
    seq = (2, 3)        # RESET_SAFE_STOP → SERVO_ON

for code in seq:
    call(cctrl, SetRobotControl.Request(robot_control=int(code)), cto=2.0)
    time.sleep(0.4)                    # 컨트롤러가 상태를 반영할 시간

ensure_auto()                          # ★AUTONOMOUS 복귀
call(cctrl, SetRobotControl.Request(robot_control=3), cto=2.0)   # 서보온 재확인

t0 = time.time()
while time.time() - t0 < 8.0:          # 최대 8초 확인
    if robot_state() == 1 and motion_status() == 0:
        return True                    # STANDBY + IDLE = 복구 성공
    time.sleep(0.3)
return False
```

### 왜 `ensure_auto()`가 필요한가

**`movej`는 AUTONOMOUS 모드에서만 허용된다.** MANUAL이면 `5.7170`으로 거부된다.
복구 과정에서 모드가 바뀔 수 있으므로 이동 재개 전에 반드시 자율모드로 되돌린다.

### 왜 충돌감도를 안 건드리는가

```
※충돌감도는 건드리지 않는다 — 배치 시작 때 이미 고정했고,
  변경은 MANUAL 전환이 필요해 복구 흐름을 끊는다.
```

`change_collision_sensitivity`는 **AUTONOMOUS에서 `5.7170`으로 거부**된다.
바꾸려면 MANUAL로 전환해야 하는데, 그러면 방금 맞춘 자율모드가 다시 풀린다.
그래서 감도는 **배치 시작 전 딱 한 번만** 설정한다.

### 복구 직전 감속 정지

```python
def soft_stop(self):
    """일반 중단/오류는 감속 정지(DR_STO=2). E-stop을 흉내 내지 않는다."""
    rq = self.ctx.MoveStop.Request()
    rq.stop_mode = 2
    return self.ctx.call(self.ctx.cstop, rq, cto=2.0)
```

> ⚠️ **`stop_mode=2`(감속정지)를 쓰는 이유**: E-stop(0/1)을 흉내 내면 로봇이 급정지해
> 공을 쥔 채 토크 스파이크가 생기고, 오히려 새로운 알람을 유발한다.
> 테스트에서도 `set(stop_modes) == {2}` 로 이 규칙을 고정하고 있다.

---

## 5. 3단계 — 재개, 그리고 위치 기억

### 절대목표라서 가능한 재개

모든 이동은 **절대각(`mode = 0`)** 으로 보낸다.

```python
req.pos  = [float(x) for x in target]   # 절대 목표 관절각
req.mode = 0                            # MOVE_MODE_ABSOLUTE
```

따라서 **어디서 멈췄든 같은 명령을 다시 보내면** 남은 거리만 이동한다.
상대이동(`mode=1`)이었다면 재전송할 때마다 이동량이 누적되어 엉뚱한 곳으로 간다.

### 단일 이동 — 그냥 다시 보내면 된다

```python
def goto(target, profile='free', quick=False, move=None):
    """외력으로 멈추면 자동 복구 후 '같은 절대 목표'로 이어서 이동한다."""
    while True:
        try:
            return _goto_once(target, profile, quick, move)
        except TripError as exc:
            if not force_recover(jog.cycle_msg, exc):
                raise RuntimeError('외력 감지 — 자동 복구 한도 초과/실패, 원인 확인 후 재시작') from exc
```

### 🔑 여러 구간 체인 — 여기가 핵심

블렌딩 체인은 `[P5 → P9 → P24]` 처럼 **여러 목표를 순차로** 보낸다.
중간에 멈췄을 때 **처음부터 다시 보내면 앞 경유점으로 되돌아간다.**

**해결: 어디까지 명령을 보냈는지 기억한다.**

```python
def blend_chain(steps, radius=None, quick_end=False, at=None):
    at = [0] if at is None else at          # ★진행 위치 (리스트 = 참조로 공유)
    if at[0] == 0:
        # 첫 실행에서만: 이미 그 점 위에 서 있으면 건너뜀(전환 체인 시작 처리)
        keep = trim_passed_steps(steps, cur)
        at[0] = len(steps) - len(keep)
    ...
    for i in range(at[0], len(steps)):      # ★기억한 구간부터
        tgt, mk = steps[i][0], steps[i][1]
        at[0] = i                           # ★여기까지 명령을 보냈다
        send(amovej, tgt)
        if i < len(steps) - 1:
            wait_lead(tgt, lead_i)
    wait_arrival(steps[-1][0])
    ...
except TripError as exc:
    if force_recover(f'{jog.cycle_msg} (블렌딩)', exc):
        return blend_chain(steps, radius=radius, quick_end=quick_end, at=at)   # ★at 그대로 전달
```

**`at`을 리스트로 두는 이유**: 파이썬 정수는 불변이라 함수 안에서 바꿔도 밖에 안 보인다.
리스트(가변 객체)로 두면 **재귀 호출 사이에 값이 공유**된다.

### 동작 예시

```
체인 [P5, P9, P24] 를 실행 중

  at=0 → P5 로 명령 발사 ─┐
                          │  P5 로 가는 도중 사람이 밀어서 정지
                          ▼
  트립 감지 → 복구 → blend_chain(steps, at=[0])
  at=0 → P5 로 다시 발사 (절대목표라 남은 거리만) → P9 → P24  ✅

────────────────────────────────────────────────────────

  at=0 → P5 발사 → 근접 → at=1 → P9 발사 ─┐
                                           │  P9 로 가는 도중 정지
                                           ▼
  트립 감지 → 복구 → blend_chain(steps, at=[1])
  at=1 → P9 부터 재개 → P24  ✅  (P5 로 되돌아가지 않음)
```

### 상대이동은 특별 취급 — `movel_z`

무게 측정의 `+Z 100mm`는 **상대이동**이라 그냥 재시도하면 총 이동이 늘어난다.
그래서 **실제로 간 거리를 실측 누적**해 남은 거리만 다시 간다.

```python
def movel_z(dz_mm, vmm=20.0):
    moved = [0.0]
    def once():
        z0 = _cur_z()                        # posx[2] 실측
        try:
            _movel_z_once(dz_mm - moved[0], vmm)     # 남은 거리만
        finally:
            z1 = _cur_z()
            if z0 is not None and z1 is not None:
                moved[0] += (z1 - z0)        # 실제 이동량 누적
        return True
    return trip.guarded(once)
```

### 스플라인만은 예외

`MoveSplineJoint`는 여러 점을 **한 명령으로** 넘기므로 컨트롤러 안에서 어디까지 갔는지 알 수 없다.
복구 후에는 **P5부터 순서대로 다시** 간다(절대목표라 중복 이동 없음).

```python
except TripError as exc:
    if not force_recover(f'{jog.cycle_msg} (P5 복귀)', exc):
        raise RuntimeError(...)
    # 스플라인은 항상 시작점(P5)부터 다시 그리므로 goto 로 남은 구간을 잇는다
    for tgt, mk in ((common, 'ord_return'), (next_lift, 'ord_to_lift')):
        goto(tgt, 'free', quick=True, move=mk)
```

---

## 6. 위치 추정의 함정

### 실측 사고 (2026-07-28)

처음엔 "위치를 보고 지나왔는지 판단"하려 했다.

```python
# ❌ 잘못된 추정
if dist(다음점, 현재) < dist(다음점, 경유점):
    이미 그 구간에 들어섰다 → 앞 경유점 버림
```

**증상**: 뚜껑 체인 `[P5, P9, P24]`를 P21에서 시작했더니 **P5·P9가 통째로 잘려 P24로 직행**.
회피 경유가 무효가 되어 **충돌 위험**이었다.

### 원인 — 관절공간은 일직선이 아니다

실제 웨이포인트로 검산하면 바로 드러난다:

```
P5 → P9  거리 = 78.7°       ← 경유점에서 다음 점까지
P21 → P9 거리 = 45.6°       ← 현재 위치에서 다음 점까지  (더 가깝다!)

→ "P5를 지나왔다"고 오판 → P5 삭제
   실제로는 P21에 서 있었을 뿐, P5 근처에도 안 갔다
```

**직교공간(x,y,z)이라면** 세 점이 대략 한 직선 위에 있을 때만 이 판정이 성립한다.
**관절공간(6축 각도)** 에서는 그런 보장이 전혀 없다.

### 교훈

> **로봇의 진행 상태는 추정하지 말고 기록한다.**
> 기하학적 추론은 관절공간에서 배신한다.

지금 `trim_passed_steps()`에는 **추정 로직을 아예 제거**했고, 사고 경위를 주석에 남겼다.
남은 기능은 "**이미 그 점 위에 서 있으면**(0.5° 이내) 건너뛴다" 하나뿐이다.

```python
def trim_passed_steps(steps, cur, near_deg=0.5):
    """체인 시작 시 '이미 그 점 위에 서 있는' 선행 경유점만 잘라낸다.

    ★위치로 '지나왔는지'를 추정하지 않는다. 관절공간 경유점은 일직선이 아니라서
      '다음 점에 더 가깝다' 같은 판정은 안 지나온 점까지 잘라낸다.
      실측 사고(2026-07-28): 뚜껑 체인 [P5, P9, P24]를 P21에서 시작했더니 그 추정으로
      P5·P9가 통째로 잘려 P24로 직행했다 — 회피 경유가 무효가 되어 충돌 위험.
      외력 복구 후 '어디부터 이어갈지'는 추정이 아니라 어디까지 명령을 보냈는지
      (blend_chain 의 at 인덱스)로 정한다.
    """
```

### 검증 결과

| 상황 | 경로 |
|---|---|
| P21에서 뚜껑 체인 시작 | `P5 → P9 → P24` ✅ 전부 유지 |
| P5 **위에 서서** 시작 | `P9 → P24` ✅ 제자리 재이동만 생략 |
| P9로 가던 중 외력 정지 | `P9 → P24` ✅ 기억한 지점부터 |

---

## 7. 적용 범위

**작업 전 과정에 적용**했다. 예전엔 주문 사이클에만 있었다.

| 시퀀스 | 대상 동작 | 자동복구 |
|---|---|---|
| **`order.py`** (주문 파지) | `goto` · `blend_chain` · 스플라인 복귀 | ✅ |
| **`lid.py`** (뚜껑) | `goto` · `blend_chain` · `movel_z`(통 들기) | ✅ |
| `lid.py` | `movel_abs` (**순응 안착**) | ⛔ [8장](#8-자동복구-금지-구역) |
| **`waste.py`** (폐기물) | `chain` · `_one` | ✅ |

### 함께 넣은 것 — 이동 중 상태 감시

`lid.py`·`waste.py`에는 **이동 중 상태 감시가 아예 없었다.**
외력으로 멈춰도 도착 타임아웃(45~60초)까지 그냥 서 있었다.

```python
if time.time() - last_st > 2.0:
    last_st = time.time()
    trip.check()     # ★2초마다 안전상태 확인
```

`order.py`에 있던 이 감시를 세 시퀀스 전부에 넣었다.

---

## 8. 자동복구 금지 구역

### 순응제어 구간 (뚜껑 안착 · `movel_abs`)

```python
# ★순응 구간은 자동복구하지 않는다: 안전정지로 순응이 풀린 뒤 같은 목표로 다시
#   보내면 뚜껑을 '강성'으로 밀어 넣는다. 작업자가 확인하는 편이 안전하다.
if in_comp[0]:
    if st in (3, 5, 8, 9, 10):
        raise RuntimeError(
            f'순응 구간(뚜껑 안착) 이동 중 안전정지(state={st}) — '
            '자동복구 금지 구간, 뚜껑 상태 확인 후 수동 재시작')
else:
    trip.check(st)
```

**왜 위험한가:**

```
순응제어 ON  →  로봇이 부드럽게 뚜껑을 안착시키는 중
      │
      ▼  외력 또는 과부하로 보호정지
안전정지 발생  →  ★순응제어가 함께 풀린다
      │
      ▼  자동복구 후 같은 목표로 MoveL 재전송
로봇이 강성(stiff) 상태로 뚜껑을 밀어 넣음  →  뚜껑·통 파손
```

순응 상태를 `in_comp` 플래그로 추적해 이 구간만 제외한다.

```python
in_comp = [False]

def comp_on():
    ...
    in_comp[0] = True

def comp_off():
    ...
    in_comp[0] = False
```

### 비상정지 · NOT_READY

```python
DEAD_STATES = (6, 15)     # EMERGENCY_STOP, NOT_READY
if st in DEAD_STATES:
    raise RuntimeError(f'복구 금지 로봇 상태({st})')
```

비상정지는 **사람이 물리 버튼을 눌러 만든 상태**다. 소프트웨어가 임의로 풀면 안 된다.

---

## 9. 한도와 포기

무한 재시도는 위험하다. **같은 자리에서 계속 걸리면 진짜 장애물**일 수 있다.

```python
'trip_recover_n': 2,      # 시퀀스 실행 1회당 자동복구 허용 횟수
```

```python
def recover(self, where, exc=None):
    why = '외력 감지' if (exc is None or getattr(exc, 'is_external_force', True)) else '안전정지'
    al  = getattr(exc, 'alarm', None) or self.alarm()

    if self.left <= 0:
        self.msg(f'🛑 {why} 반복({self.total}회) — 자동 복구 중단, 원인 확인 필요')
        return False                    # ← 포기

    self.left -= 1
    n = self.total - self.left
    self.msg(f'🛡 {why} — 정지 (알람 {al}) · 자동 복구 {n}/{self.total} · {where}')
    self.soft_stop()
    time.sleep(0.4)

    rec = getattr(self.ctx, 'recover_trip', None)
    if not callable(rec) or not rec():
        self.msg(f'⚠️ 자동 복구 실패 — 수동 확인 필요 ({where})')
        return False                    # ← 복구 자체 실패

    self.msg(f'🔧 복구 완료 — 기억한 위치부터 이어서 진행 · {where}')
    return True
```

### 포기했을 때의 안전 보장

복구에 실패하면 **로봇에 이동 명령을 더 보내지 않는다.**

```python
raise RuntimeError('외력 감지 — 자동 복구 한도 초과/실패, 원인 확인 후 재시작')
```

이 규칙은 테스트로 고정되어 있다:

```python
def test_robot_safe_stop_aborts_without_sending_next_movej(self):
    ...
    # ★핵심: 복구에 실패했으면 로봇에 이동 명령을 더 보내지 않는다.
    self.assertEqual(len(ctx.move_requests), 3)
    # 정지는 항상 감속정지(2)만 — E-stop(0/1)을 흉내 내지 않는다.
    self.assertEqual(set(ctx.stop_modes), {2})
```

### 한도의 범위

`trip_recover_n`은 **시퀀스 실행 1회당**이다.
주문 사이클에서 2번 쓰고, 뚜껑 시퀀스에서 또 2번 쓸 수 있다.
같은 사이클 안에서 같은 자리를 3번 걸리면 그때 포기한다.

---

## 10. UI 표시 3단계

백엔드가 붙인 **접두 이모지**로 상태를 구분한다. 문구가 길어져도 안 깨진다.

| 상태 | 메시지 형식 | 상단 알약 | 히어로 카드 |
|---|---|---|---|
| **복구 중** | `🛡 외력 감지 — 정지 (알람 5.7173) · 자동 복구 1/2 · ④ 공 잡기` | 🟡 `외력 감지 · 복구 중` | 앰버 `외력 감지` |
| **재개** | `🔧 복구 완료 — 기억한 위치부터 이어서 진행 · ④ 공 잡기` | 🟢 `복구 완료 · 재개` | 초록 `작업 재개` |
| **포기** | `🛑 외력 감지 반복(2회) — 자동 복구 중단, 원인 확인 필요` | 🔴 `외력 감지 · 복구 중단` | 빨강 `복구 중단` |
| 복구 실패 | `⚠️ 자동 복구 실패 — 수동 확인 필요 (④ 공 잡기)` | 🔴 `외력 감지 · 복구 중단` | 빨강 |

### 오탐 방지

`🛡 충돌감도 설정 중`(배치 시작 메시지)도 같은 이모지를 쓴다.
그래서 **접두만 보지 않고 문구까지** 확인한다.

```javascript
const _fxOn   = _m.indexOf('🛡 외력 감지')===0 || _m.indexOf('🛡 안전정지')===0;
const _fxDone = _m.indexOf('🔧 복구 완료')===0;
const _fxHalt = _m.indexOf('🛑')===0 || _m.includes('자동 복구 실패')
                                     || _m.includes('자동 복구 한도');
```

> ⚠️ **문구 형식을 바꾸면 UI가 깨진다.** `TripRecovery` 클래스 주석에도 경고를 남겨 두었다.

---

## 11. 파라미터 전체

### 자동복구 (`config/motion.py` → `MOTION`)

| 키 | 값 | 의미 |
|---|---|---|
| `trip_recover_n` | **2** | 시퀀스 1회당 자동복구 허용 횟수 (0이면 자동복구 없이 즉시 중단) |

### 충돌감도 (`config/motion.py` → `COLLISION`)

| 키 | 값 | 의미 |
|---|---|---|
| `fixed` | **20** | 1~100, **높을수록 민감**. 배치 시작 때 한 번만 설정 |

> 10은 손으로 밀어도 잘 안 걸릴 만큼 둔감했다. 너무 올리면 공을 쥔 채 감속할 때
> 토크 스파이크를 충돌로 오인(`5.7173`)한다.

### 상태 감시 주기

| 항목 | 값 |
|---|---|
| 이동 중 상태 확인 | **2.0** s |
| 복구 코드 간 간격 | 0.4 s |
| 복구 완료 확인 상한 | 8.0 s |
| 감속정지 모드 | `stop_mode = 2` |

### 안전 알람 참고

| 코드 | 의미 | 조치 |
|---|---|---|
| `5.7060` | 서보온 자가검사 실패(payload/공구 불일치) | 펜던트에서 공구 지정. **코드에서 `set_tool` 호출 금지** |
| `5.7170` | AUTONOMOUS에서 충돌감도 변경 시도 | MANUAL에서만 변경 가능 |
| `5.7173` | 충돌 감지 | 감도 확인 후 복구 |

---

## 12. 시퀀스 다이어그램

```
사람이 팔을 민다
       │
       ▼
로봇 컨트롤러: 보호정지 (state 5, 알람 5.7173)
       │
       │  ≤2초 이내
       ▼
trip.check()  ──▶  raise TripError(5, '5.7173')
       │
       ▼
TripRecovery.recover()
       │
       ├─ 한도 확인 (남은 횟수 > 0?) ──── 아니오 ──▶ 🛑 중단, 이동명령 보내지 않음
       │  예
       ├─ msg('🛡 외력 감지 — 정지 (알람 5.7173) · 자동 복구 1/2 · ④ 공 잡기')
       ├─ soft_stop(stop_mode=2)            감속정지
       ├─ sleep(0.4)
       │
       ▼
ctx.recover_trip()  =  core/jog.py recover_from_trip()
       │
       ├─ state=5 → seq = (4, 7, 3)
       │     RECOVERY_SAFE_STOP → sleep(0.4)
       │     RESET_RECOVERY     → sleep(0.4)
       │     SERVO_ON           → sleep(0.4)
       ├─ ensure_auto()                     AUTONOMOUS 복귀
       ├─ SERVO_ON 재확인
       └─ 8초 안에 STANDBY + IDLE?  ── 아니오 ──▶ False
              예
       ▼
msg('🔧 복구 완료 — 기억한 위치부터 이어서 진행 · ④ 공 잡기')
       │
       ▼
재개:  blend_chain(steps, at=at)        ← 기억한 구간부터
       또는  _goto_once(target, ...)     ← 같은 절대 목표
       │
       ▼
아무 일 없었던 듯 작업 계속
```

---

## 13. 코드 위치

| 구성요소 | 파일 | 역할 |
|---|---|---|
| `TripError` | `sequences/common.py` | 트립 예외 + 외력 여부 판별 |
| `TripRecovery` | `sequences/common.py` | 감지·한도·복구 호출·재시도 (`check`/`trip_if_stopped`/`recover`/`guarded`/`soft_stop`) |
| `trim_passed_steps()` | `sequences/common.py` | 시작 시 '서 있는 점'만 건너뛰기 (추정 없음) |
| `recover_from_trip()` | `core/jog.py` | **실제 복구** — 상태별 DRFL 제어코드 |
| `ensure_auto()` | `core/jog.py` | AUTONOMOUS 복귀 |
| `blend_chain(at=)` | `sequences/order.py` · `lid.py` | 진행 위치 기억 + 재개 |
| `chain(at=)` | `sequences/waste.py` | 동일 |
| `movel_z(moved=)` | `sequences/lid.py` | 상대이동 잔여거리 보정 |
| 상태 표시 | `palpa_ui.html` `render()` | 3단계 UI |
| 회귀 테스트 | `tests/test_order_routing.py` | 복구·재개·중단 규칙 고정 |

### 관련 테스트

```python
test_robot_safe_stop_aborts_without_sending_next_movej
    복구 실패 시 이동명령을 더 보내지 않는다 · 정지는 감속(2)만

test_external_force_recovers_and_resumes_the_same_move
    복구 성공 시 같은 이동을 이어서 끝까지 간다
```

---

## 요약 — 세 가지 원칙

| # | 원칙 | 이유 |
|---|---|---|
| 1 | **절대목표로만 이동한다** (`mode=0`) | 어디서 멈춰도 재전송하면 남은 거리만 간다 |
| 2 | **진행 위치는 기억한다** (`at` 인덱스) | 위치로 추정하면 관절공간에서 배신한다 |
| 3 | **못 풀면 손을 뗀다** (한도 2회) | 진짜 장애물일 때 밀어붙이면 더 위험하다 |
