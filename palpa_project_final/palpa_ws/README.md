# PALPA 로봇 제어 인터페이스 안내

이 문서는 **로봇 제어팀**에게 전달하는 통합용 문서입니다.
백엔드(FastAPI)/프론트엔드 쪽 로직은 몰라도 되고, 아래 인터페이스대로만 구현하시면
`main_controller_node`, 백엔드와 자동으로 통합됩니다.

Claude(AI)로 이 코드를 이어서 작업하실 경우, 이 README와 `palpa_control/robot_controller_node.py`를
같이 붙여넣고 "TODO 부분을 실제 로봇 제어 코드로 채워달라"고 요청하시면 됩니다.

---

## 1. 패키지 구성

```
palpa_ws/src/
├── palpa_interfaces/          # 공용 인터페이스 정의 (수정 시 팀 전체 합의 필요)
│   ├── action/ProcessItem.action
│   └── msg/
│       ├── InspectionResult.msg
│       └── RobotStatus.msg
└── palpa_control/
    └── palpa_control/
        ├── main_controller_node.py         # (완료) 백엔드 주문 수신 -> goal 전송 -> 결과 회신
        ├── robot_controller_node.py        # ⭐ 로봇 제어팀이 채워야 할 파일
        ├── robot_controller_stub_node.py   # (완료) robot_controller_node 대신 쓰는 시뮬레이션 버전
        └── exception_handler_node.py       # (완료) 로봇 에러/ESTOP 처리
```

## 2. 담당 범위

**로봇 제어팀이 작업할 파일은 `robot_controller_node.py` 하나입니다.**
그 안의 `_execute_pipeline()` 함수에 있는 TODO 부분만 실제 Doosan M0609 + RG2 그리퍼
제어 코드로 채워주시면 됩니다. 액션 서버 등록, goal 수신, feedback/result 반환 구조는
이미 구현되어 있으니 건드릴 필요 없습니다.

## 3. 노드 구성 및 역할

| 노드 | 담당 | 역할 |
|---|---|---|
| `main_controller_node` | (완료) | 백엔드로부터 주문 수신, item별 threshold 결정 후 액션 goal 전송, 결과를 백엔드로 회신 |
| `robot_controller_node` | **로봇 제어팀** | 실제 로봇 동작 (픽업 → 무게측정 → 지름측정 → 압축(탄성측정) → 판정 → 포장/리젝트) |
| `robot_controller_stub_node` | (완료, 통합테스트용) | 실제 로봇 없이 랜덤 pass/fail로 동일 인터페이스 시뮬레이션 |
| `exception_handler_node` | (완료) | 로봇 ERROR/ESTOP 상태 감지 시 해당 item을 실패 처리하여 백엔드가 무한 대기하지 않도록 함 |

## 4. 토픽 / 액션 목록

| 이름 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/order/new` | `std_msgs/String` (JSON) | 백엔드 → `main_controller_node` | 신규 주문 알림 (기존, 변경 없음) |
| `/palpa/process_item` | `palpa_interfaces/action/ProcessItem` | `main_controller_node` → `robot_controller_node` | **신규.** 아이템 1개 검사 요청 (action) |
| `/inspection/result` | `palpa_interfaces/msg/InspectionResult` | `main_controller_node` / `exception_handler_node` → 백엔드 | **신규.** 검사 완료/실패 결과. 백엔드가 구독해서 주문 상태(DB) 업데이트 |
| `/robot/status` | `palpa_interfaces/msg/RobotStatus` | `robot_controller_node` → `exception_handler_node` | **신규.** 로봇 현재 상태(IDLE/BUSY/ERROR/ESTOPPED) 보고 |

> 백엔드 담당자에게는 `/inspection/result`를 구독해서 주문 상태를 갱신하는 로직 추가가
> 별도로 필요합니다 (이번 통합 범위에는 로봇 제어 쪽만 포함).

## 5. ProcessItem.action 인터페이스

```
# Goal (main_controller -> robot_controller)
string order_id
string item_id
string item_type
float32 weight_threshold_min / weight_threshold_max
float32 diameter_threshold_min / diameter_threshold_max
float32 elasticity_threshold_min / elasticity_threshold_max
---
# Result (robot_controller -> main_controller)
bool success
string item_id
string final_stage        # PACKING 또는 REJECTING
float32 measured_weight
float32 measured_diameter
float32 measured_elasticity
string reject_reason      # 실패 시에만
---
# Feedback (진행 중 계속 publish)
string item_id
string current_stage      # PICKING / WEIGHING / MEASURING_DIAMETER / COMPRESSING / CLASSIFYING / PACKING / REJECTING
float32 progress          # 0.0 ~ 1.0
```

**로봇 제어팀이 하실 일**: goal로 받은 threshold 값과, 실제 측정한
`measured_weight` / `measured_diameter` / `measured_elasticity` 를 비교해서
`CLASSIFYING` 단계에서 합격/불합격을 판정하고, `PACKING` 또는 `REJECTING`으로
이동시키는 로직을 작성하시면 됩니다. 판정 로직 자체(스켈레톤)는 이미 작성되어 있고,
실제 측정값을 채워넣는 부분(TODO 3곳)만 로봇 코드로 연결하시면 됩니다.

## 6. 빌드 및 실행

```bash
cd ~/ws_cobot_pjt/ws_dsr   # 기존 cobot 워크스페이스 (또는 팀에서 쓰는 경로)
# palpa_interfaces, palpa_control 폴더를 src/ 아래로 복사한 뒤
colcon build --packages-select palpa_interfaces palpa_control
source install/setup.bash

# 실제 로봇 연결 후 (로봇팀 작업 완료 시)
ros2 run palpa_control robot_controller_node

# 실제 로봇 없이 통합 테스트만 할 때
ros2 run palpa_control robot_controller_stub_node

# 항상 같이 실행
ros2 run palpa_control main_controller_node
ros2 run palpa_control exception_handler_node
```

> 기존 `dsr_control2`/`dsr_common2` 패키지와 같은 워크스페이스에 두실 경우
> `ROS_DOMAIN_ID`, `PYTHONPATH` 격리 이슈가 있었던 것으로 알고 있어서
> (`~/ros2_ws` vs `~/ws_cobot_pjt/ws_dsr`), 새 패키지를 어느 워크스페이스에
> 넣을지는 기존 환경 설정과 맞춰서 결정해주세요.

## 7. 기존 대비 변경/신규 사항 요약

- **신규**: `/palpa/process_item` 액션, `/inspection/result` 토픽, `/robot/status` 토픽
- **신규 노드**: `robot_controller_node`(로봇팀 구현 대상), `exception_handler_node`
- **기존 유지**: `/order/new` 토픽, `main_controller_node`의 threshold 디스패치 로직
- **미정 (확정 필요)**:
  - `ITEM_THRESHOLDS` 값 (현재 `main_controller_node.py`에 임시값으로 하드코딩됨, QA 기준 확정되면 교체)
  - `main_controller_node`가 `/order/new`에서 파싱하는 `item_type` 필드명 (현재 `item.get('id')`로 임시 사용 중, 프론트/백엔드 payload 스펙 확정 필요)
  - ESTOP 발생 시 재시도 여부/정책

## 8. 질문 있으실 때

인터페이스(액션/토픽 필드) 자체를 바꿔야 하는 경우, `palpa_interfaces` 패키지는
`main_controller_node`와 공유되므로 **혼자 수정하지 마시고** 먼저 공유해주세요.
`robot_controller_node.py` 내부 구현(TODO 부분)은 자유롭게 수정하셔도 됩니다.
