"""
paths.py — 런타임 데이터 파일 위치 단일 소유
=============================================
[역할] 웨이포인트·임계값·속도프로파일·측정CSV 가 어디에 저장/로드될지 한 곳에서 결정한다.

왜 필요한가:
  예전에는 각 모듈이 '~/ws_cobot_pjt/ws_dsr/waypoints.json' 처럼 **홈 절대경로**를
  직접 박아 썼다. 그러면 git clone 만 해서는 웨이포인트(P1~P30)가 통째로 없어
  로봇이 어디로 갈지 모르는 상태가 된다. 여기서 프로젝트 안 data/ 를 기본값으로
  잡아 저장소에 동봉된 데이터로 바로 돌아가게 한다.

우선순위:
  1) 환경변수 PALPA_DATA_DIR 이 있으면 그 폴더        (다른 로봇/현장별 데이터 분리용)
  2) 없으면 이 저장소의 data/                          (기본 — 클론 후 즉시 실행 가능)

  예)  PALPA_DATA_DIR=~/my_robot_data python3 grip_web.py

주의: 여기 정의된 파일들은 실행 중 **덮어쓰기**된다(웨이포인트 저장·임계값 자동계산·
      측정 기록 등). data/ 안의 값은 '공장 초기값'이 아니라 마지막 저장 상태다.
"""
import os

# 이 파일이 있는 곳 = 프로젝트 루트
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

DATA_DIR = os.path.expanduser(os.environ.get('PALPA_DATA_DIR', '')) or os.path.join(ROOT_DIR, 'data')
os.makedirs(DATA_DIR, exist_ok=True)


def data_path(name):
    """data 폴더 안 파일의 절대경로."""
    return os.path.join(DATA_DIR, name)


WAYPOINTS_FILE = data_path('waypoints.json')             # 티칭한 P1~P30 관절각
SEQUENCES_FILE = data_path('sequences.json')             # 블록 프로그램(시퀀스 탭)
SPEED_FILE     = data_path('speed_profile.json')         # 이동별 속도 · 전역 배속(속도 탭)
THRESHOLD_FILE = data_path('ball_thresholds.json')       # 공 판별 임계값(자동계산 결과)
MEASURE_CSV    = data_path('ball_measurements_final.csv')  # 측정 이력(학습 데이터)
