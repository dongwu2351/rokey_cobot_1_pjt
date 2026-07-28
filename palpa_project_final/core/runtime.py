"""프로세스 전역 공유 상태.

STATE/ROBOT/JOG/JOINTS 는 main()에서 생성되어 여러 모듈이 함께 본다.
모듈을 나누면 `from core.runtime import STATE` 는 그 시점의 None 을 복사해 버리므로,
반드시 `from core import runtime as rt` 후 **rt.STATE** 처럼 속성으로 접근해야 한다.
(WAYPOINTS/PROGRAMS 는 clear()/extend() 등 제자리 갱신만 하므로 직접 import 해도 안전.)
"""
# 자동 분리: grip_web.py → core/ (내용 동일, 위치만 이동)

import os
import threading
import paths                       # 데이터 파일 위치(저장소 동봉 data/)

STATE_NAMES = {0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF', 4: 'TEACHING',
               5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMING', 8: 'RECOVERY',
               9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY'}
RCLPY_LOCK = threading.Lock()      # rclpy.init() 중복 방지(모니터/조그 스레드 공용)
MOTION_ACTIVE = threading.Event()  # 로봇 이동 중이면 set() → 모니터 폴링 일시정지(모션 방해 방지)

WAYPOINTS = []                     # 티칭 포인트: [{'name','posj','gripper','note'}]
WP_LOCK = threading.Lock()
WP_FILE = paths.WAYPOINTS_FILE

PROGRAMS = {}                      # {name: [block,...]}
PROG_LOCK = threading.Lock()
PROG_FILE = paths.SEQUENCES_FILE

JOG_SPEED = 15.0                   # jog 속도[%] (single jog = 250mm/s × speed%)

# ── main() 이 채운다. 다른 모듈은 반드시 rt.STATE 형태로 접근할 것 ──────────
STATE = None                       # GripperState
ROBOT = None                       # RobotMonitor
JOG = None                         # JogWorker
JOINTS = None                      # JointSub
