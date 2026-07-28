#!/usr/bin/env python3
"""
RG2 그리퍼 + M0609 통합 대시보드 — 브라우저 GUI (실기기) · 진입점

tkinter가 ROS+Wayland에서 세그폴트를 내므로 GUI를 로컬 웹페이지로 제공.
파이썬은 화면을 안 그리고 브라우저가 그림 → 충돌 없음. 표준 라이브러리만 사용.

구성(기능별 모듈):
    core/runtime.py      프로세스 공유 상태(STATE/ROBOT/JOG/JOINTS · 락 · 웨이포인트)
    core/rg2.py          RG2 하드웨어 계층(상수 · 너비↔각도 · Modbus TCP)
    core/store.py        웨이포인트 / 블록 프로그램 파일 입출력
    core/gripper.py      GripperState — 그리퍼 소켓 I/O · 측정 · 판별 · CSV
    core/monitor.py      JointSub · RobotMonitor — 관절/상태/힘 구독
    core/jog.py          JogWorker — 모션 스레드(조그 · 주문사이클 · 블록 · 폐기물)
    core/api.py          HTTP Handler — /api 엔드포인트, 페이지 서빙
    core/legacy_page.py  내장 폴백 UI(palpa_ui.html 없을 때만)

    grip_config.py       설정 퍼사드 → config/{motion,classify,waypoints,gripper}.py
    grip_cycle.py        시퀀스 퍼사드 → sequences/{order,lid,waste,common}.py
    palpa_ui.html        실제 대시보드 UI(요청 시마다 새로 읽어 서빙)

실행:
    python3 grip_web.py                 # 기본 192.168.1.1, 포트 8760
    python3 grip_web.py 192.168.1.1 65 8760
그 후 브라우저에서  http://localhost:8760  접속.
"""
import os
import sys
from http.server import ThreadingHTTPServer

import grip_config as cfg
from core import runtime as RT
from core.api import Handler
from core.gripper import GripperState
from core.jog import JogWorker
from core.monitor import JointSub, RobotMonitor
from core.store import load_programs, load_waypoints


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else '192.168.1.1'
    unit = int(sys.argv[2]) if len(sys.argv) > 2 else 65
    port = int(sys.argv[3]) if len(sys.argv) > 3 else 8760

    load_waypoints()
    load_programs()                                 # 저장된 시퀀스(함수) 복원
    _sp_op = cfg.load_speed_profile()               # 저장된 구간속도/전역배속 복원(속도탭)
    if _sp_op is not None:
        print(f'[speed] speed_profile.json 적용 · 구간 {{s:cfg.SPEED[s+"_max"] for s in cfg.SPEED_SEGMENTS}} · op {_sp_op}')
    if cfg.load_thresholds():                       # 자동계산된 임계값 있으면 적용
        print(f'[classify] ball_thresholds.json 적용됨: {cfg.CLASSIFY}')

    # ★워커 인스턴스는 runtime 모듈에 담는다. 다른 모듈이 RT.STATE 로 같은 객체를 본다.
    RT.STATE = GripperState(host, unit); RT.STATE.start()
    _n = RT.STATE.load_saved()                       # CSV 측정이력 복원(재시작해도 유지)
    if _n:
        print(f'[measure] ball_measurements_final.csv 에서 {_n}개 측정 복원')
    RT.JOINTS = JointSub(); RT.JOINTS.start()
    RT.ROBOT = RobotMonitor(); RT.ROBOT.start()
    RT.JOG = JogWorker(); RT.JOG.start()

    srv = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    url = f'http://localhost:{port}'
    dom = os.environ.get('ROS_DOMAIN_ID', '(미설정=0)')
    print(f'\n  ✅ 대시보드 실행 중  →  브라우저에서 {url}  접속   (종료: Ctrl-C)')
    print(f'  🔗 ROS_DOMAIN_ID = {dom}   ← 팔 드라이버와 같아야 로봇 데이터가 보입니다\n')
    # webbrowser.open 제거: 헤드리스/백그라운드에서 자식프로세스 대기(do_wait)로 멈추는 문제. 브라우저는 직접 여세요.
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        RT.STATE.stop(); RT.ROBOT.stop(); RT.JOG.stop()
        if RT.JOINTS: RT.JOINTS.stop()
        srv.shutdown()
        print('\n종료합니다.')


if __name__ == '__main__':
    main()
