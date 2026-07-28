"""내장 폴백 UI — palpa_ui.html 이 없을 때만 서빙된다."""
# 자동 분리: grip_web.py → core/ (내용 동일, 위치만 이동)



# ── HTML ─────────────────────────────────────────────────────────────────────
PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RG2 · M0609 Control</title>
<style>
:root{
 --bg:#0d1117; --card:#161b22; --card2:#1c2430; --line:#2a3441;
 --txt:#e6edf3; --mut:#8b949e; --accent:#2dd4bf; --accent2:#38bdf8;
 --warn:#f87171; --amber:#fbbf24; --good:#34d399;
}
*{box-sizing:border-box}
body{margin:0;background:
   radial-gradient(1200px 600px at 100% -10%, #10233022, transparent),
   radial-gradient(900px 500px at -10% 110%, #0e2a2622, transparent), var(--bg);
 color:var(--txt);font-family:system-ui,'Noto Sans KR',sans-serif;padding:22px;min-height:100vh}
.head{display:flex;align-items:center;gap:12px;margin:0 auto 18px;max-width:960px}
.head h1{font-size:19px;margin:0;font-weight:800;letter-spacing:.3px}
.head .sub{color:var(--mut);font-size:13px}
.pill{margin-left:auto;display:flex;gap:8px}
.badge{font-size:12px;font-weight:700;padding:5px 11px;border-radius:999px;border:1px solid var(--line);
 background:#0b1220;color:var(--mut);display:flex;align-items:center;gap:6px}
.dot{width:8px;height:8px;border-radius:50%;background:var(--mut)}
.dot.on{background:var(--good);box-shadow:0 0 8px var(--good)}
.dot.off{background:var(--warn)}
.dot.mv{background:var(--amber);box-shadow:0 0 8px var(--amber)}
.wrap{display:grid;grid-template-columns:360px 1fr;gap:18px;max-width:960px;margin:0 auto}
@media(max-width:820px){.wrap{grid-template-columns:1fr}}
.card{background:linear-gradient(180deg,var(--card),#12171f);border:1px solid var(--line);
 border-radius:16px;padding:18px;box-shadow:0 10px 30px #0006}
h2{margin:0 0 14px;font-size:13px;color:var(--accent);text-transform:uppercase;letter-spacing:1.2px;font-weight:800}
canvas{background:radial-gradient(120px 80px at 50% 20%,#1b2430,#0c1017);border-radius:12px;width:100%;display:block}
.big{font-size:15px;font-weight:800;margin:12px 0 6px}
.detect-on{color:var(--warn)}.detect-off{color:var(--mut)}
label{font-size:12px;color:var(--mut);display:flex;justify-content:space-between}
.val{color:var(--accent);font-weight:800}
input[type=range]{width:100%;-webkit-appearance:none;height:6px;border-radius:6px;
 background:linear-gradient(90deg,var(--accent),var(--accent2));outline:none;margin:6px 0 12px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;
 background:#e6edf3;border:3px solid var(--accent);cursor:pointer;box-shadow:0 2px 6px #000a}
.btns{display:flex;flex-wrap:wrap;gap:8px}
button{flex:1;min-width:80px;background:#1f2937;color:var(--txt);border:1px solid var(--line);border-radius:10px;
 padding:10px;cursor:pointer;font-size:13px;font-weight:700;transition:.15s}
button:hover{border-color:var(--accent);transform:translateY(-1px)}
button.acc{background:linear-gradient(180deg,#0ea5a0,#0b7d78);border:0}
button.warn{background:linear-gradient(180deg,#dc2626,#991b1b);border:0}
button.amber{background:linear-gradient(180deg,#d97706,#92400e);border:0}
.row{display:flex;justify-content:space-between;padding:6px 0;font-size:14px;border-bottom:1px solid #ffffff08}
.row b{color:var(--txt);font-weight:700}
.kpi{display:flex;gap:12px;margin:4px 0 12px}
.kpi .box{flex:1;background:var(--card2);border:1px solid var(--line);border-radius:12px;padding:10px 12px}
.kpi .box .l{font-size:11px;color:var(--mut)}
.kpi .box .v{font-size:22px;font-weight:800;color:var(--accent);margin-top:2px}
.kpi .box .v.blue{color:var(--accent2)}
.cap{color:var(--accent2);font-size:11px;letter-spacing:.5px;margin:12px 0 6px;font-weight:700}
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.grid6{display:grid;grid-template-columns:repeat(6,1fr);gap:6px}
.cell{background:var(--card2);border:1px solid var(--line);border-radius:9px;padding:7px 4px;text-align:center}
.cell .l{color:var(--mut);font-size:10px}
.cell .v{font-weight:800;font-size:14px;margin-top:2px}
pre{background:#0a0e14;color:#7ee787;font-size:10.5px;padding:10px;border-radius:10px;overflow-x:auto;
 white-space:pre-wrap;border:1px solid var(--line);margin:0}
#gstatus{margin-top:10px;font-size:12px;color:var(--mut)}
.state-pill{padding:5px 12px;border-radius:999px;font-weight:800;font-size:13px;display:inline-block}
button:active{transform:translateY(1px)}
button:disabled{cursor:default}
button.busy{opacity:.65}
button.busy::after{content:'';position:absolute;right:9px;top:50%;width:12px;height:12px;margin-top:-7px;
 border:2px solid #ffffff55;border-top-color:#fff;border-radius:50%;animation:spin .6s linear infinite}
button{position:relative}
button.flash{animation:flash .5s}
@keyframes flash{0%{box-shadow:0 0 0 0 var(--accent)}100%{box-shadow:0 0 0 8px transparent}}
@keyframes spin{to{transform:rotate(360deg)}}
#toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%) translateY(24px);opacity:0;
 background:linear-gradient(180deg,#1f2937,#141a22);border:1px solid var(--accent);color:var(--txt);
 padding:12px 22px;border-radius:12px;font-weight:700;font-size:14px;pointer-events:none;z-index:50;
 transition:.28s cubic-bezier(.2,.9,.3,1.2);box-shadow:0 10px 30px #000b}
#toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
button small{font-size:9px;color:#ffffffaa;font-weight:600}
.jogwrap{display:grid;grid-template-columns:1fr 1fr;gap:18px}
@media(max-width:820px){.jogwrap{grid-template-columns:1fr}}
.joggrid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.jogitem{display:flex;align-items:center;gap:6px;background:var(--card2);border:1px solid var(--line);
 border-radius:10px;padding:6px 8px}
.jogitem .ax{width:30px;font-weight:800;color:var(--accent2);font-size:13px;text-align:center}
.jogitem button{flex:1;padding:9px 0;font-size:17px;min-width:0;line-height:1}
.jogitem button:active{background:var(--accent);color:#04110f}
.sldrow{display:flex;align-items:center;gap:8px;margin:5px 0}
.sldrow .sl{width:34px;font-weight:800;color:var(--accent2);font-size:13px;text-align:center}
.sldrow input[type=range]{flex:1;margin:0;height:6px}
.sldrow b{width:60px;text-align:right;color:var(--txt);font-size:13px;font-variant-numeric:tabular-nums}
</style></head><body>
<div id="toast"></div>
<div class="head">
 <h1>🦾 RG2 · M0609 <span class="sub">Control Dashboard</span></h1>
 <div class="pill">
   <span class="badge"><span class="dot" id="g_dot"></span>그리퍼</span>
   <span class="badge"><span class="dot" id="r_dot"></span>로봇</span>
 </div>
</div>
<div class="wrap">
 <div class="card">
  <h2>그리퍼 제어</h2>
  <canvas id="cv" width="320" height="180"></canvas>
  <div id="detect" class="big detect-off">⚪ 제품 미감지</div>
  <label>너비 <span class="val"><span id="wlab">60</span> mm</span></label>
  <input type="range" id="w" min="0" max="110" step="1" value="60">
  <label>설정 파지력 <span class="val"><span id="flab">20</span> N</span></label>
  <input type="range" id="f" min="3" max="40" step="1" value="20">
  <div class="btns" style="margin-bottom:8px">
    <button id="g_close">➖ 조이기 <small>1mm·꾹</small></button>
    <button id="g_open">➕ 벌리기 <small>1mm·꾹</small></button>
  </div>
  <div class="btns">
    <button class="acc" onclick="setW(110)">완전 열기</button>
    <button class="acc" onclick="setW(0)">완전 닫기</button>
    <button class="warn" onclick="recover(event)">🔧 복구</button>
  </div>
  <div id="gstatus">연결 중...</div>
  <div class="cap" style="margin-top:12px">🔬 공 검증 측정 (힘–너비 곡선)</div>
  <button class="acc" onclick="startMeasure(event)" style="width:100%">공 측정 시작 (5N 접촉→40N 강성)</button>
  <div id="measure_status" style="font-size:12px;color:var(--amber);margin-top:5px;min-height:16px"></div>
  <canvas id="curve" width="308" height="120" style="margin-top:6px;background:#161616;border-radius:8px;display:block"></canvas>
  <div id="measure_result" style="font-size:13px;margin-top:5px;line-height:1.5"></div>
  <div id="save_row" style="display:none;flex-direction:column;gap:5px;margin-top:8px">
    <div style="display:flex;gap:5px">
      <button onclick="saveMeas('유압')" style="flex:1;background:#0f3d2e;color:#34d399;border:1px solid #34d399;border-radius:8px;padding:8px 4px;font-weight:800;cursor:pointer;font-size:12px">🟢 유압<br>(정상)</button>
      <button onclick="saveMeas('구멍')" style="flex:1;background:#3d1414;color:#f87171;border:1px solid #f87171;border-radius:8px;padding:8px 4px;font-weight:800;cursor:pointer;font-size:12px">🔴 구멍<br>(불량)</button>
      <button onclick="saveMeas('무압')" style="flex:1;background:#12233d;color:#58a6ff;border:1px solid #58a6ff;border-radius:8px;padding:8px 4px;font-weight:800;cursor:pointer;font-size:12px">🔵 무압<br>(정상)</button>
    </div>
    <div style="display:flex;gap:5px">
      <button onclick="saveMeas('하드')" style="flex:1;background:#3d3210;color:#facc15;border:1px solid #facc15;border-radius:8px;padding:8px 4px;font-weight:800;cursor:pointer;font-size:12px">🟡 하드<br>(야구공)</button>
      <button onclick="saveMeas('소프트')" style="flex:1;background:#3d2610;color:#fb923c;border:1px solid #fb923c;border-radius:8px;padding:8px 4px;font-weight:800;cursor:pointer;font-size:12px">🟠 소프트<br>(야구공)</button>
    </div>
  </div>
  <div style="display:flex;justify-content:space-between;align-items:center;margin-top:10px">
    <div class="cap" style="margin:0">📋 저장된 측정 <span id="save_cnt" style="color:var(--accent2)">0</span>개</div>
    <a href="/api/measure_csv" style="font-size:12px;color:#58a6ff;text-decoration:none">⬇ CSV 다운로드</a>
  </div>
  <div id="save_table" style="font-size:11px;margin-top:4px;max-height:150px;overflow-y:auto"></div>

  <div class="cap" style="margin-top:14px">🤖 자동 작업 (잡기→경유→판별→분류)</div>
  <div style="display:flex;gap:8px;align-items:center;margin-bottom:6px;font-size:12px;color:var(--mut)">
    <span>작업속도</span>
    <input type="range" id="wspd" min="5" max="40" step="5" value="40" style="flex:1;margin:0" oninput="document.getElementById('wspdlab').textContent=this.value">
    <b class="val"><span id="wspdlab">40</span></b>
  </div>
  <div style="display:flex;gap:8px;align-items:center;margin-bottom:6px;font-size:12px;color:var(--mut)">
    <span>전체속도%</span>
    <input type="range" id="opspd" min="10" max="40" step="5" value="20" style="flex:1;margin:0"
      oninput="document.getElementById('opspdlab').textContent=this.value"
      onchange="fetch('/api/op_speed?v='+this.value).then(()=>toast('전체속도 '+this.value+'%'))">
    <b class="val"><span id="opspdlab">20</span>%</b>
  </div>
  <div style="display:flex;gap:8px">
    <button class="acc" onclick="startWork(event)" style="flex:2;font-size:16px;padding:12px">▶ 작업 시작</button>
    <button class="warn" onclick="stopWork(event)" style="flex:1;font-size:15px;padding:12px">■ 중단</button>
  </div>
  <div id="cycle_box" style="display:none;background:#1a1f27;border-radius:8px;padding:10px;margin-top:8px">
    <div id="cycle_msg" style="font-size:15px;font-weight:800;color:#fbbf24">대기</div>
    <div id="cycle_class" style="font-size:20px;font-weight:900;margin-top:6px"></div>
  </div>
  <div style="font-size:11px;color:#6b7280;margin-top:5px">홈→경유→잡는위치→경유→판별→(포장/불량)→홈 · 무압/정상=포장, 구멍=불량</div>
 </div>

 <div class="card">
  <div style="display:flex;align-items:center;gap:10px;margin-bottom:6px">
    <h2 style="margin:0">로봇 · 무게 / 외력</h2>
    <span class="state-pill" id="statepill" style="margin-left:auto;background:#30363d;color:#8b949e">-</span>
  </div>
  <div id="robot_msg" style="color:var(--mut);font-size:13px;margin:6px 0">로봇 연결 확인 중...</div>

  <div class="kpi">
    <div class="box"><div class="l">🏋 물체 무게 (Fz 영점기준)</div><div class="v" id="k_west">-</div></div>
    <div class="box"><div class="l">💪 툴 외력 |F|</div><div class="v blue" id="k_fmag">-</div></div>
  </div>
  <div class="row"><span>총 매달린 무게(그리퍼+물체)</span><b id="i_wdsr">-</b></div>

  <div class="cap">툴 외력 (N · BASE)</div>
  <div class="grid3">
   <div class="cell"><div class="l">Fx</div><div class="v" id="fx">-</div></div>
   <div class="cell"><div class="l">Fy</div><div class="v" id="fy">-</div></div>
   <div class="cell"><div class="l">Fz</div><div class="v" id="fz">-</div></div>
  </div>
  <div class="cap">툴 모멘트 (Nm · TOOL)</div>
  <div class="grid3">
   <div class="cell"><div class="l">Tx</div><div class="v" id="tx">-</div></div>
   <div class="cell"><div class="l">Ty</div><div class="v" id="ty">-</div></div>
   <div class="cell"><div class="l">Tz</div><div class="v" id="tz">-</div></div>
  </div>
  <div class="cap">관절 외력토크 J1~J6 (Nm)</div>
  <div class="grid6" id="ext"></div>
  <div class="cap">관절 토크센서 J1~J6 (Nm)</div>
  <div class="grid6" id="jts"></div>

  <div class="btns" style="margin-top:14px">
    <button class="acc" onclick="servoOn(event)">⚡ 서보 ON</button>
    <button class="amber" onclick="servoOff(event)">⏻ 서보 OFF</button>
    <button class="warn" onclick="robotRecover(event)">🛡 로봇 복구</button>
    <button onclick="tare(event)">⚖ 무게 영점</button>
  </div>
  <div style="font-size:11px;color:var(--mut);margin-top:4px">외력·충돌로 보호정지(빨강)되면 <b>🛡로봇 복구</b> → 서보 ON 순서로 복구</div>

  <div class="cap" style="margin-top:14px">그리퍼 원시 레지스터 (258~)</div>
  <pre id="regs">-</pre>
 </div>

 <div class="card" style="grid-column:1/-1">
  <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap">
   <h2 style="margin:0">🕹 로봇 이동 (슬라이더 드래그)</h2>
   <span style="color:var(--mut);font-size:12px">드래그 후 놓으면 그 위치로 이동 · <b style="color:var(--amber)">서보 ON 필요</b> · 한 번에 최대 40°/80mm</span>
   <button class="acc" style="margin-left:auto;flex:0 0 auto;min-width:80px" onclick="goHome(event)">🏠 홈</button>
   <button class="warn" style="flex:0 0 auto;min-width:80px" onclick="motionStop(event)">■ 즉시정지</button>
  </div>
  <div style="font-size:11px;color:var(--amber);margin:-2px 0 6px">⚠ 보호정지 시 자동 재이동하지 않습니다. 원인과 경로를 확인한 뒤 수동 복구하세요.</div>
  <div style="display:flex;align-items:center;gap:10px;margin:10px 0 6px;flex-wrap:wrap;font-size:12px;color:var(--mut)">
   <span>이동 속도</span>
   <input type="range" id="jspd" min="5" max="12" step="1" value="10" style="flex:1;max-width:200px;margin:0">
   <b class="val"><span id="jspdlab">10</span></b>
   <button onclick="authYield(event)" style="margin-left:14px;background:#2e2708;color:#fbbf24;border:1px solid #fbbf24;border-radius:8px;padding:7px 10px;font-weight:700;cursor:pointer">🖐 펜던트로 넘기기(티칭)</button>
   <button onclick="authReclaim(event)" style="background:#0f2e26;color:#34d399;border:1px solid #34d399;border-radius:8px;padding:7px 10px;font-weight:700;cursor:pointer">🔄 ROS로 회수(작업)</button>
   <span style="margin-left:14px">충돌 민감도(낮을수록 둔감)</span>
   <input type="range" id="collis" min="10" max="10" step="1" value="10" disabled style="flex:1;max-width:140px;margin:0">
   <b class="val"><span id="collislab">10</span></b>
  </div>
  <div class="jogwrap">
   <div>
    <div class="cap">관절 J1~J6 (°) — 드래그해서 이동</div>
    <div id="joint_sliders"></div>
   </div>
   <div>
    <div class="cap">작업축 BASE (X,Y,Z mm · Rx,Ry,Rz °) — 드래그해서 이동</div>
    <div id="task_sliders"></div>
   </div>
  </div>
 </div>

 <div class="card" style="grid-column:1/-1">
  <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap">
   <h2 style="margin:0">🎯 포인트 티칭 (DART 방식)</h2>
   <span style="color:var(--mut);font-size:12px">🖐 자유이동으로 자세 잡고 → 저장. 순서대로 쌓임. 파일로 받아서 AI에게 전달.</span>
  </div>
  <div style="display:flex;gap:8px;align-items:center;margin:10px 0;flex-wrap:wrap">
   <input id="wp_name" placeholder="포인트 이름(예: 파지접근)" style="flex:1;min-width:130px;padding:9px;border-radius:8px;border:1px solid #2a3441;background:#12161c;color:#e6edf3">
   <input id="wp_note" placeholder="메모(예: 공 위 5cm)" style="flex:2;min-width:150px;padding:9px;border-radius:8px;border:1px solid #2a3441;background:#12161c;color:#e6edf3">
   <button class="acc" style="min-width:150px" onclick="wpSave(event)">📍 현재 자세 저장</button>
  </div>
  <div style="display:flex;justify-content:space-between;align-items:center">
   <div class="cap" style="margin:0">저장된 포인트 <span id="wp_cnt" style="color:var(--accent2)">0</span>개</div>
   <div style="display:flex;gap:12px">
     <a href="/api/wp_download" style="font-size:12px;color:#58a6ff;text-decoration:none">⬇ 파일 다운로드</a>
     <a onclick="wpClear()" style="font-size:12px;color:#f87171;text-decoration:none;cursor:pointer">🗑 전체삭제</a>
   </div>
  </div>
  <div id="wp_table" style="font-size:11.5px;margin-top:6px"></div>
 </div>
</div>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
const W=document.getElementById('w'),F=document.getElementById('f');
const MAXW=110;
function draw(actual,detected,moving){
 const w=cv.width,h=cv.height,cx=w/2,half=(actual/MAXW)*100+6,by=30,ty=h-34,ft=15;
 ctx.clearRect(0,0,w,h);
 ctx.fillStyle='#334155';ctx.fillRect(cx-60,12,120,by-12);
 const col=moving?'#fbbf24':'#2dd4bf';
 for(const s of [-1,1]){const fx=cx+s*half;
  ctx.fillStyle=col;ctx.fillRect(fx-ft/2,by,ft,ty-by);
  ctx.fillStyle='#a78bfa';ctx.fillRect(fx-ft/2,ty-20,ft,20);}
 if(detected){ctx.fillStyle='#f59e0b';ctx.fillRect(cx-half+ft/2,ty-20,2*half-ft,20);
  ctx.fillStyle='#111';ctx.font='11px sans-serif';ctx.textAlign='center';ctx.fillText('물체',cx,ty-6);}
 ctx.fillStyle='#e6edf3';ctx.font='bold 15px sans-serif';ctx.textAlign='center';
 ctx.fillText(actual.toFixed(0)+' mm',cx,ty+22);
}
let st=null;
function pushW(){document.getElementById('wlab').textContent=W.value;
 clearTimeout(st);st=setTimeout(()=>fetch('/api/set?width='+W.value),50);draw(+W.value,false,false);}
function pushF(){document.getElementById('flab').textContent=F.value;fetch('/api/force?n='+F.value);}
W.oninput=pushW;F.oninput=pushF;
function setW(v){W.value=v;pushW();}
function toast(msg,ms){const t=document.getElementById('toast');t.textContent=msg;t.classList.add('show');
 clearTimeout(t._t);t._t=setTimeout(()=>t.classList.remove('show'),ms||1800);}
async function act(url,label,ev){
 const btn=ev&&ev.currentTarget;
 if(btn){btn.classList.add('busy','flash');btn.disabled=true;}
 toast('⏳ '+label+' 실행 중...',3000);
 try{await fetch(url);toast('✅ '+label+' 전송됨 · 상태를 확인하세요');}
 catch(e){toast('❌ '+label+' 실패');}
 finally{if(btn){setTimeout(()=>{btn.classList.remove('busy','flash');btn.disabled=false;},600);}}
}
function recover(ev){act('/api/recover','그리퍼 복구',ev);}
function tare(ev){act('/api/tare','무게 영점',ev);}
function servoOn(ev){act('/api/servo_on','서보 ON',ev);}
function servoOff(ev){act('/api/servo_off','서보 OFF',ev);}
function robotRecover(ev){act('/api/robot_recover','로봇 복구',ev);}
// 꾹 누르면 연속 실행되는 버튼
function holdBtn(el,fn,ms){let iv;
 const go=(e)=>{if(e){e.preventDefault();}fn();clearInterval(iv);iv=setInterval(fn,ms||120);};
 const stop=()=>clearInterval(iv);
 el.addEventListener('mousedown',go);el.addEventListener('touchstart',go,{passive:false});
 ['mouseup','mouseleave','touchend','touchcancel'].forEach(e=>el.addEventListener(e,stop));}
// 그리퍼 1mm 미세 조작 (클라이언트에서 슬라이더 갱신 + 전송)
function nudge(d){W.value=Math.max(0,Math.min(110,(+W.value)+d));pushW();}
// 로봇 조그 (부드러운 연속 · 누르는 동안 흐르고 떼면 정지)
function jogSpeed(){return document.getElementById('jspd').value;}
// 드래그 슬라이더로 로봇 이동
const JLIM=[[-180,180],[-95,95],[-150,150],[-180,180],[-180,180],[-180,180]];
const TLIM=[[-800,800],[-800,800],[-100,1000],[-180,180],[-180,180],[-180,180]];
const TNAME=['X','Y','Z','Rx','Ry','Rz'];
const dragActive={};   // 사용자가 드래그 중인 슬라이더는 실시간 값으로 덮어쓰지 않음
function sliderRow(id,label,lo,hi,onCommit){
 const w=document.createElement('div');w.className='sldrow';
 const sp=document.createElement('span');sp.className='sl';sp.textContent=label;
 const s=document.createElement('input');s.type='range';s.min=lo;s.max=hi;s.step=0.5;s.id=id;s.value=0;
 const b=document.createElement('b');b.id=id+'v';b.textContent='-';
 s.addEventListener('input',()=>{dragActive[id]=true;b.textContent=(+s.value).toFixed(1);});
 s.addEventListener('change',()=>{onCommit(+s.value);setTimeout(()=>{dragActive[id]=false;},1500);});
 w.appendChild(sp);w.appendChild(s);w.appendChild(b);return w;}
function moveJoint(j,val){fetch('/api/movej?j='+j+'&val='+val+'&vel='+jogSpeed());toast('J'+(j+1)+' → '+val.toFixed(1)+'° 이동');}
function moveTask(a,val){fetch('/api/movel?ax='+a+'&val='+val+'&vel='+jogSpeed());toast(TNAME[a]+' → '+val.toFixed(1)+' 이동');}
function buildSliders(){
 const jc=document.getElementById('joint_sliders');
 for(let j=0;j<6;j++)jc.appendChild(sliderRow('j'+j,'J'+(j+1),JLIM[j][0],JLIM[j][1],(v)=>moveJoint(j,v)));
 const tc=document.getElementById('task_sliders');
 for(let a=0;a<6;a++)tc.appendChild(sliderRow('t'+a,TNAME[a],TLIM[a][0],TLIM[a][1],(v)=>moveTask(a,v)));}
function updateSliders(rb){
 if(rb.posj)for(let j=0;j<6;j++){if(!dragActive['j'+j]){const s=document.getElementById('j'+j);if(s){s.value=rb.posj[j];document.getElementById('j'+jv(j)).textContent=rb.posj[j].toFixed(1);}}}
 if(rb.posx)for(let a=0;a<6;a++){if(!dragActive['t'+a]){const s=document.getElementById('t'+a);if(s){s.value=rb.posx[a];document.getElementById('t'+a+'v').textContent=rb.posx[a].toFixed(1);}}}
}
function jv(j){return j+'v';}
function motionStop(ev){act('/api/motion_stop','로봇 즉시정지',ev);}
function goHome(ev){act('/api/home','홈자세 이동',ev);}
function startMeasure(ev){act('/api/measure','공 측정',ev);}
function doCalib(ev){const v=document.getElementById('calibmm').value;fetch('/api/calib?real='+v);toast('크기 보정 적용: 실제 '+v+'mm 기준. 다시 측정하세요');}
function drawCurve(mr){
 const c=document.getElementById('curve'),x=c.getContext('2d'),W=c.width,H=c.height;
 x.clearRect(0,0,W,H);
 const ML=46,MR=14,MT=16,MB=22,pw=W-ML-MR,ph=H-MT-MB;
 x.strokeStyle='#3a3d41';x.lineWidth=1;x.strokeRect(ML,MT,pw,ph);
 x.font='10px sans-serif';
 if(!mr||!mr.points||!mr.points.length){x.fillStyle='#888';x.textAlign='center';x.fillText('측정 대기',ML+pw/2,MT+ph/2);return;}
 const pts=mr.points,ws=pts.map(p=>p.width),fs=pts.map(p=>p.force);
 let wlo=Math.min(...ws),whi=Math.max(...ws);
 if(whi-wlo<3){const m=(wlo+whi)/2;wlo=m-1.5;whi=m+1.5;}else{const pad=(whi-wlo)*0.15;wlo-=pad;whi+=pad;}
 const fhi=Math.max(42,...fs);
 const px=f=>ML+f/fhi*pw, py=w=>MT+(1-(w-wlo)/(whi-wlo))*ph;
 const many=pts.length>12;   // 연속 램프(39점) 여부
 // Y축 눈금값(너비)
 x.textAlign='right';
 for(let i=0;i<=4;i++){const w=wlo+(whi-wlo)*i/4,Y=py(w);
  x.strokeStyle='#20242b';x.beginPath();x.moveTo(ML,Y);x.lineTo(ML+pw,Y);x.stroke();
  x.fillStyle='#9aa3ad';x.fillText(w.toFixed(1),ML-4,Y+3);}
 // X축 눈금값(힘) — 점 많으면 10N 간격만
 x.textAlign='center';x.fillStyle='#9aa3ad';
 if(many){[10,20,30,40].forEach(f=>x.fillText(f+'N',px(f),H-7));}
 else{fs.forEach(f=>x.fillText(f+'N',px(f),H-7));}
 // 접촉점 세로선
 if(mr.contact_f){x.strokeStyle='#f0883e';x.lineWidth=1;x.setLineDash([4,3]);
  x.beginPath();x.moveTo(px(mr.contact_f),MT);x.lineTo(px(mr.contact_f),MT+ph);x.stroke();
  x.setLineDash([]);x.fillStyle='#f0883e';x.textAlign='center';
  x.fillText('접촉 '+mr.contact_f+'N',px(mr.contact_f),MT+10);}
 // 곡선
 x.strokeStyle='#2dd4bf';x.lineWidth=2;x.beginPath();
 pts.forEach((p,i)=>{const X=px(p.force),Y=py(p.width);i?x.lineTo(X,Y):x.moveTo(X,Y);});
 x.stroke();
 // 점(연속이면 작게, 라벨 생략) / 이산이면 값 표시
 pts.forEach((p,i)=>{const X=px(p.force),Y=py(p.width);
  x.fillStyle='#4ec9b0';x.beginPath();x.arc(X,Y,many?1.6:3.5,0,7);x.fill();
  if(!many){x.fillStyle='#e6edf3';x.textAlign='center';x.fillText(p.width.toFixed(1),X,Y-7);}});
 // 시작/끝 너비만 라벨(연속 모드)
 if(many){x.fillStyle='#e6edf3';x.textAlign='left';x.fillText(ws[0].toFixed(1),px(fs[0])+3,py(ws[0])-4);
  x.textAlign='right';x.fillText(ws[ws.length-1].toFixed(1),px(fs[fs.length-1])-3,py(ws[ws.length-1])+12);}
 // 축 제목
 x.fillStyle='#9cdcfe';x.textAlign='left';x.fillText('너비 mm',2,MT-4);
}
function renderMeasure(d){
 const st=document.getElementById('measure_status'),rs=document.getElementById('measure_result');
 if(d.measuring){st.textContent='측정중... '+(d.measure_msg||'');}
 else{st.textContent=d.measure_msg==='완료'?'':(d.measure_msg||'');}
 const mr=d.measure_result;drawCurve(mr);
 if(mr){const p0=mr.points[0],pN=mr.points[mr.points.length-1];
  const tbl='시작 '+p0.force+'N→'+p0.width.toFixed(1)+'mm  ·  끝 '+pN.force+'N→'+pN.width.toFixed(1)+'mm'
   +(mr.contact_f?('  ·  접촉 '+mr.contact_f+'N'):'');
  let dt=(mr.detail||[]).map(l=>'<div style="color:#aab2bd;font-size:12px">'+l+'</div>').join('');
  rs.innerHTML='<div style="color:#7ee787;font-size:11px;margin-bottom:4px">'+tbl+'</div>'
   +'<div style="background:#1a1f27;border-radius:8px;padding:8px;margin-top:2px">'
   +'<b style="color:#fbbf24;font-size:14px">▶ 판정: '+mr.class+'</b>'
   +'<div style="margin-top:5px">'+dt+'</div></div>';}
 // 저장 버튼: 측정결과 있을 때만
 document.getElementById('save_row').style.display=(mr&&mr.points&&mr.points.length)?'flex':'none';
 // 저장 이력 테이블
 const sv=d.saved||[];document.getElementById('save_cnt').textContent=sv.length;
 const tb=document.getElementById('save_table');
 if(sv.length){let h='<table style="width:100%;border-collapse:collapse;font-size:11px"><tr style="color:#8b949e">'
   +'<td>#</td><td>라벨</td><td>크기</td><td>40N폭</td><td>총눌림</td></tr>';
  sv.forEach((r,i)=>{const c=r.label==='유압'?'#34d399':r.label==='구멍'?'#f87171':r.label==='무압'?'#58a6ff':r.label==='하드'?'#facc15':r.label==='소프트'?'#fb923c':'#8b949e';
   h+='<tr style="border-top:1px solid #222"><td>'+(i+1)+'</td><td style="color:'+c+';font-weight:700">'+r.label+'</td>'
    +'<td>'+r.size+'</td><td>'+r.w40+'</td><td><b>'+r.comp_total+'</b></td></tr>';});
  tb.innerHTML=h+'</table>';}
 else{tb.innerHTML='<div style="color:#6b7280;font-size:11px">아직 저장 없음</div>';}
}
function saveMeas(label){fetch('/api/measure_save?label='+encodeURIComponent(label))
 .then(r=>r.json()).then(d=>toast(d.msg||'저장'));}
function startWork(ev){const v=document.getElementById('wspd').value;fetch('/api/start_work?vel='+v).then(r=>r.json()).then(d=>toast(d.msg||'작업'));}
function stopWork(ev){fetch('/api/stop_work').then(r=>r.json()).then(d=>toast(d.msg||'중단'));}
function renderCycle(c){const box=document.getElementById('cycle_box');if(!box)return;
 if(c&&(c.active||c.msg)){box.style.display='block';
  document.getElementById('cycle_msg').textContent=c.msg||'';
  const cl=document.getElementById('cycle_class');
  if(c.flow_class){cl.textContent=c.flow_class;
   cl.style.color=c.flow_class.indexOf('불량')>=0?'#f87171':c.flow_class.indexOf('무압')>=0?'#58a6ff':'#34d399';}
  else cl.textContent='';}
 else box.style.display='none';}
function setCollision(el){document.getElementById('collislab').textContent='10';toast('충돌 민감도는 배치 시작 시 10으로 고정 설정됩니다');}
function authYield(ev){fetch('/api/compliance?on=1');toast('🖐 제어권 넘기기 ON — 펜던트에서 "강제회수" 누르면 넘어갑니다. 그다음 핸드가이딩!');}
function authReclaim(ev){fetch('/api/compliance?on=0');toast('🔄 제어권 ROS로 회수 + 서보온. 이제 작업(이동/판별) 가능');}
function toggleCompliance(el){fetch('/api/compliance?on='+(el.checked?1:0));
 toast(el.checked?'🖐 티칭모드 ON — RT 정지+백드라이브. 손으로 미세요! (E-stop 준비, 안 밀리면 알려주세요)':'티칭모드 OFF — RT 재개+서보ON (다시 단단해짐)');}
function wireGripper(){holdBtn(document.getElementById('g_close'),()=>nudge(-1),60);
 holdBtn(document.getElementById('g_open'),()=>nudge(1),60);}
function T(id,v){document.getElementById(id).textContent=v;}
function fill(id,arr){const g=document.getElementById(id);
 g.innerHTML=(arr||[0,0,0,0,0,0]).map((v,i)=>'<div class="cell"><div class="l">J'+(i+1)+'</div><div class="v">'+(arr?v.toFixed(1):'-')+'</div></div>').join('');}
function dot(id,cls){document.getElementById(id).className='dot '+cls;}
const STATE_COLOR={1:['#0f2e26','#34d399'],2:['#2e2708','#fbbf24'],3:['#2e0f0f','#f87171'],5:['#2e0f0f','#f87171'],6:['#2e0f0f','#f87171']};
async function poll(){
 try{const d=await (await fetch('/api/status')).json();
  document.getElementById('gstatus').textContent=d.status;
  const dt=document.getElementById('detect');
  const badge='display:block;text-align:center;padding:10px;border-radius:10px;font-size:16px;font-weight:900;margin:2px 0';
  if(d.grip){dt.innerHTML='✅ 접촉되었습니다 (파지 감지)<br><span style="font-size:13px;font-weight:600">폭 '+d.actual+'mm · '+d.force+'N로 파지 중</span>';
    dt.style.cssText=badge+';background:linear-gradient(180deg,#0f3d2e,#0a2a20);color:#34d399;border:1px solid #34d399';}
  else if(d.detected){dt.innerHTML='✅ 물체 감지됨<br><span style="font-size:13px;font-weight:600">폭 '+d.actual+'mm · 두께≈'+d.thickness+'mm · '+d.force+'N로 파지</span>';
    dt.style.cssText=badge+';background:linear-gradient(180deg,#0f3d2e,#0a2a20);color:#34d399;border:1px solid #34d399';}
  else{dt.innerHTML='⚪ 물체 없음 <span style="font-size:12px;font-weight:600">(폭 '+d.actual+'mm)</span>';
    dt.style.cssText=badge+';background:#1a1f27;color:#8b949e;border:1px solid #2a3441';}
  draw(d.actual,d.detected,!!d.busy);
  dot('g_dot', d.connected?'on':'off');
  let rg='';d.regs.forEach((v,i)=>{rg+='['+String(i).padStart(2)+']='+String(v).padStart(5)+(i%6==5?'\n':' ');});
  document.getElementById('regs').textContent=rg;
  renderMeasure(d);
  renderRobot(d.robot||{});
  renderWaypoints(d.waypoints||[]);
  renderCycle(d.cycle);
 }catch(e){document.getElementById('gstatus').textContent='서버 연결 대기...';}
}
function wpSave(ev){const n=document.getElementById('wp_name').value,m=document.getElementById('wp_note').value;
 fetch('/api/wp_save?name='+encodeURIComponent(n)+'&note='+encodeURIComponent(m))
  .then(r=>r.json()).then(d=>{toast(d.msg||'저장');document.getElementById('wp_name').value='';document.getElementById('wp_note').value='';});}
function wpGoto(i){if(confirm('P'+(i+1)+' 자세로 이동할까요? (서보 ON 필요)'))fetch('/api/wp_goto?i='+i).then(()=>toast('P'+(i+1)+'로 이동'));}
function wpDelete(i){fetch('/api/wp_delete?i='+i).then(()=>toast('삭제됨'));}
function wpClear(){if(confirm('모든 포인트를 삭제할까요?'))fetch('/api/wp_clear').then(()=>toast('전체삭제'));}
function wpRole(i,v){fetch('/api/wp_update?i='+i+'&role='+encodeURIComponent(v)).then(()=>toast('역할 지정됨'));}
function wpEdit(i,field){const cur=(window.WPS&&window.WPS[i])?(window.WPS[i][field]||''):'';
 const v=prompt((field==='name'?'포인트 이름':'메모')+' 수정:',cur);
 if(v!==null)fetch('/api/wp_update?i='+i+'&'+field+'='+encodeURIComponent(v)).then(()=>toast('수정됨'));}
function renderWaypoints(wps){
 window.WPS=wps;
 document.getElementById('wp_cnt').textContent=wps.length;
 const tb=document.getElementById('wp_table');
 if(!wps.length){tb.innerHTML='<div style="color:#6b7280">아직 저장된 포인트 없음 — 자세 잡고 📍저장</div>';return;}
 let h='<table style="width:100%;border-collapse:collapse"><tr style="color:#8b949e"><td>#</td><td>이름 ✎</td><td>관절 J1~J6 (°)</td><td>좌표 XYZ·ABC (mm/°)</td><td>그리퍼</td><td>역할</td><td>메모 ✎</td><td></td></tr>';
 wps.forEach((w,i)=>{h+='<tr style="border-top:1px solid #222">'
   +'<td style="color:#58a6ff;font-weight:700">'+(i+1)+'</td>'
   +'<td style="color:#e6edf3;font-weight:600;cursor:pointer" onclick="wpEdit('+i+',\'name\')" title="클릭해 수정">'+(w.name||'-')+' ✎</td>'
   +'<td style="font-family:monospace;color:#9cdcfe">['+w.posj.map(v=>v.toFixed(1)).join(', ')+']</td>'
   +'<td style="font-family:monospace;color:'+(w.posx?'#c3a6ff':'#6b7280')+'">'+(w.posx?'['+w.posx.map(v=>v.toFixed(1)).join(', ')+']':'(posx 없음 · 재티칭)')+'</td>'
   +'<td>'+w.gripper+'mm</td>'
   +'<td><select onchange="wpRole('+i+',this.value)" style="background:#1a1f27;color:#e6edf3;border:1px solid #2a3441;border-radius:4px;font-size:11px">'+['','pick','via','via2','pack','defect','home'].map(r=>'<option value="'+r+'"'+((w.role||'')===r?' selected':'')+'>'+({'':'(자동)','pick':'잡는위치','via':'경유지','via2':'집은후경유','pack':'포장','defect':'불량','home':'홈'}[r])+'</option>').join('')+'</select></td>'
   +'<td style="color:#aab2bd;cursor:pointer" onclick="wpEdit('+i+',\'note\')" title="클릭해 수정">'+(w.note||'(메모)')+' ✎</td>'
   +'<td style="white-space:nowrap"><a onclick="wpGoto('+i+')" style="color:#34d399;cursor:pointer;margin-right:8px">▶이동</a>'
   +'<a onclick="wpDelete('+i+')" style="color:#f87171;cursor:pointer">✕</a></td></tr>';});
 tb.innerHTML=h+'</table>';
}
function renderRobot(rb){
 const msg=document.getElementById('robot_msg'),pill=document.getElementById('statepill');
 if(!rb.ros){dot('r_dot','off');pill.textContent='ROS 미탑재';msg.style.display='block';
   msg.textContent='ROS 미탑재 — 워크스페이스 source 후 실행';return;}
 // 상태 pill
 const nm=rb.state_name||'-';pill.textContent=nm;
 const c=STATE_COLOR[rb.state]||['#30363d','#8b949e'];pill.style.background=c[0];pill.style.color=c[1];
 dot('r_dot', rb.connected?(rb.state===2?'mv':'on'):'off');
 if(!rb.connected){msg.style.display='block';
   msg.innerHTML='로봇 미연결 — 팔 드라이버 실행 필요<br><code style="color:#7ee787">ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100</code>';}
 else{msg.style.display='none';}
 // 물체무게(영점기준) 우선, 없으면 총 매달린무게 표시 — 안정적(평활화)
 if(rb.weight_est!=null){T('k_west',rb.weight_est.toFixed(3)+' kg');}
 else if(rb.baseline){T('k_west','0.000 kg');}
 else{T('k_west','영점 필요');}
 T('i_wdsr', rb.weight_total!=null?rb.weight_total.toFixed(3)+' kg (총)':'--');
 const f=rb.tool_force;
 if(f){const mag=Math.hypot(f[0],f[1],f[2]);
  const ax=['X','Y','Z'],av=[Math.abs(f[0]),Math.abs(f[1]),Math.abs(f[2])];
  const mi=av.indexOf(Math.max(...av));const dir=(f[mi]>=0?'+':'−')+ax[mi];
  T('k_fmag',mag.toFixed(1)+' N  ('+dir+' 방향)');
  ['fx','fy','fz'].forEach((id,i)=>{const e=document.getElementById(id);e.textContent=f[i].toFixed(1);
    e.style.color=(i===mi&&mag>3)?'#ff5555':'';e.style.fontWeight=(i===mi&&mag>3)?'900':'';});
  T('tx',f[3].toFixed(2));T('ty',f[4].toFixed(2));T('tz',f[5].toFixed(2));}
 else{T('k_fmag','--');['fx','fy','fz','tx','ty','tz'].forEach(i=>T(i,'-'));}
 fill('ext',rb.ext_torque);fill('jts',rb.joint_torque);
 updateSliders(rb);
}
fill('ext',null);fill('jts',null);draw(60,false,false);
buildSliders();wireGripper();
document.getElementById('jspd').oninput=function(){document.getElementById('jspdlab').textContent=this.value;};
setInterval(poll,220);
</script></body></html>"""
