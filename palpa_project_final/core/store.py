"""웨이포인트 · 저장 시퀀스(블록 프로그램) 파일 입출력."""
# 자동 분리: grip_web.py → core/ (내용 동일, 위치만 이동)

import json

from core.runtime import (WAYPOINTS, WP_LOCK, WP_FILE,
                          PROGRAMS, PROG_LOCK, PROG_FILE)


def save_waypoints():
    try:
        with WP_LOCK:
            data = list(WAYPOINTS)
        with open(WP_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False)
    except Exception:
        pass

def load_waypoints():
    try:
        with open(WP_FILE, encoding='utf-8') as f:
            data = json.load(f)
        with WP_LOCK:
            WAYPOINTS.clear(); WAYPOINTS.extend(data)
    except Exception:
        pass

# ── 저장 시퀀스(=함수 라이브러리): 이름 → 블록 평면리스트 ─────────────────────

def save_programs():
    try:
        with PROG_LOCK:
            data = dict(PROGRAMS)
        with open(PROG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
    except Exception:
        pass

def load_programs():
    try:
        with open(PROG_FILE, encoding='utf-8') as f:
            data = json.load(f)
        if isinstance(data, dict):
            with PROG_LOCK:
                PROGRAMS.clear(); PROGRAMS.update(data)
    except Exception:
        pass

def prog_to_text(name, blocks):
    """블록 프로그램을 사람·AI가 읽는 들여쓰기 의사코드로."""
    def desc(b):
        t = b.get('t')
        sy = '비동기' if b.get('sync', 1) == 0 else '동기'
        rad = b.get('radius', 0)
        if t == 'home':
            return f'홈 (r{rad}·{sy})'
        if t == 'move':
            wid = int(b.get('id', -1))
            if wid < 0:
                return f'이동 → 홈 (r{rad}·{sy})'
            nm = ''
            with WP_LOCK:
                if wid < len(WAYPOINTS):
                    nm = WAYPOINTS[wid].get('name', '')
            return f'이동 → P{wid + 1} {nm} (r{rad}·{sy})'
        if t == 'grip':
            w = b.get('width', 60)
            kind = '열기' if w >= 100 else ('파지' if w <= 5 else '반개방')
            return f'그리퍼 → 폭 {w}mm · 힘 {b.get("force", 20)}N ({kind})'
        if t == 'measure':
            return '판별 (공 분류: 잡고 5N→40N)'
        if t == 'wait':
            return f'대기 {b.get("sec", 1)}초'
        if t == 'movel':
            axn = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ'][int(b.get('ax', 0)) % 6]
            return f'베이스축 이동 {axn} {b.get("d", 0)}{"mm" if int(b.get("ax",0))<3 else "°"}'
        if t == 'movej1':
            return f'관절 이동 J{int(b.get("j", 0))+1} {b.get("deg", 0)}°'
        if t == 'order':
            return f'주문 설정 → {b.get("item","")} {b.get("count",1)}개'
        if t == 'pack':
            return '포장 카운트 +1 (주문품과 일치 시)'
        if t == 'loop':
            if b.get('mode') == 'order':
                return '반복 (주문 채울 때까지) {'
            return f'반복 {b.get("count", 1)}회 {{'
        if t == 'if':
            c = b.get('cond', {})
            cond = (f' {c.get("op", "or").upper()} '.join(c.get('terms', []))
                    if isinstance(c, dict) else str(c))
            return f'만약 ({cond}) {{'
        if t == 'call':
            return f'함수호출 ▶ {b.get("name", "")}'
        return {'endloop': '}', 'endif': '}', 'else': '} 아니면 {'}.get(t, str(t))
    lines = [f'# 시퀀스: {name}', '']
    ind = 0
    for b in blocks:
        t = b.get('t')
        if t in ('endloop', 'endif', 'else'):
            ind = max(0, ind - 1)
        lines.append('  ' * ind + desc(b))
        if t in ('loop', 'if', 'else'):
            ind += 1
    lines += ['', '# JSON: ' + json.dumps(blocks, ensure_ascii=False)]
    return '\n'.join(lines)
JOG_SPEED = 15.0                   # jog 속도[%] (single jog = 250mm/s × speed%)
