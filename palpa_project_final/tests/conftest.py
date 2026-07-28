"""[역할] 테스트 격리 — 테스트가 저장소의 실제 데이터를 건드리지 못하게 막는다.

일부 테스트(측정 학습·CSV 초기화 등)는 데이터 파일을 실제로 쓰고 지운다.
아무 조치 없이 두면 `data/ball_measurements_final.csv`(실측 학습 데이터)가
테스트 한 번에 날아간다. 실제로 그런 사고가 있었기에 여기서 막는다.

paths.py 가 import 시점에 PALPA_DATA_DIR 을 읽으므로, **paths 가 로드되기 전인
지금**(conftest 는 테스트 모듈보다 먼저 import 된다) 임시 폴더로 돌려놓는다.
"""
import os
import shutil
import sys
import tempfile
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]

# ★paths 를 import 하기 전에 반드시 설정되어야 한다
_TMP = tempfile.mkdtemp(prefix='palpa-test-data-')
os.environ['PALPA_DATA_DIR'] = _TMP

# 읽기만 하는 설정 파일은 사본을 넣어 준다(원본은 안전)
for _name in ('waypoints.json', 'sequences.json',
              'speed_profile.json', 'ball_thresholds.json'):
    _src = _ROOT / 'data' / _name
    if _src.exists():
        shutil.copy2(_src, Path(_TMP) / _name)

sys.path.insert(0, str(_ROOT))


def pytest_sessionfinish(session, exitstatus):
    shutil.rmtree(_TMP, ignore_errors=True)
