#!/usr/bin/env python3
"""ball_measurements_final.csv(라벨링 데이터)에서 판별 임계값을 자동 계산.
사용: python3 calibrate.py   (라벨 3종을 각 3개+ 모은 뒤 실행)
출력: 무압/유압/불량 comp 분포 + 추천 경계값 + grip_config.py에 붙일 줄."""
import csv, os, statistics as st
import paths                       # 데이터 파일 위치(저장소 동봉 data/)

CSV = paths.MEASURE_CSV
# 라벨(한글) → 분류 그룹
GROUP = {'무압': 'nopress', '딱딱': 'nopress',
         '유압': 'normal', '정상': 'normal',
         '구멍': 'defect', '불량': 'defect'}

def main():
    if not os.path.exists(CSV):
        print(f'❌ CSV 없음: {CSV}\n   웹에서 공 판정 후 라벨(🟢유압/🔴구멍/🔵무압) 버튼을 눌러 모으세요.')
        return
    data = {'nopress': [], 'normal': [], 'defect': []}
    with open(CSV, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            g = GROUP.get((row.get('label') or '').strip())
            try:
                comp = float(row.get('comp_total_5to40'))
            except (TypeError, ValueError):
                continue
            if g:
                data[g].append(comp)

    print('=' * 56)
    print('라벨별 총눌림(comp) 분포')
    print('-' * 56)
    labels = {'nopress': '🔵 무압', 'normal': '🟢 유압', 'defect': '🔴 불량'}
    for g in ('nopress', 'normal', 'defect'):
        v = sorted(data[g])
        if not v:
            print(f'{labels[g]}: (데이터 없음)')
        else:
            m = st.mean(v)
            sd = st.pstdev(v) if len(v) > 1 else 0.0
            print(f'{labels[g]}: n={len(v):2d}  범위 {min(v):.2f}~{max(v):.2f}  평균 {m:.2f}  σ {sd:.2f}  {[round(x,1) for x in v]}')

    n1, n2, n3 = data['nopress'], data['normal'], data['defect']
    if not (n1 and n2 and n3):
        print('\n⚠️ 세 종류 모두 최소 1개(권장 3개+) 필요 — 더 모으세요.')
        return

    # 경계 = 인접 그룹의 (최대, 최소) 중간점 (겹치면 평균의 중간)
    def boundary(lo, hi):
        a, b = max(lo), min(hi)
        return round((a + b) / 2, 1) if a < b else round((st.mean(lo) + st.mean(hi)) / 2, 1)

    b1 = boundary(n1, n2)   # 무압|유압
    b2 = boundary(n2, n3)   # 유압|불량
    overlap = (max(n1) >= min(n2)) or (max(n2) >= min(n3))
    print('-' * 56)
    print(f'추천 경계:  무압 ≤ {b1}  <  유압 ≤ {b2}  <  불량')
    if overlap:
        print('⚠️ 그룹 분포가 겹칩니다 — 샘플을 더 모으거나 파지 조건을 점검하세요.')
    print('\ngrip_config.py의 CLASSIFY를 이렇게:')
    print(f"    'nopress_max': {b1},")
    print(f"    'normal_max':  {b2},")
    print('=' * 56)

if __name__ == '__main__':
    main()
