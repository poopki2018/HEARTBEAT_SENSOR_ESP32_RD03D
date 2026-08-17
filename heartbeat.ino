#include <WiFi.h>
#include <WebServer.h>

// ---------------------------------------------------------------- 사용자 설정
#define RADAR_RX_PIN   16          // ESP32 RX2  <- RD-03D TXD
#define RADAR_TX_PIN   17          // ESP32 TX2  -> RD-03D RXD
#define RADAR_BAUD     256000      // RD-03D 고정 통신속도

#define USE_AP         true        // true: ESP32가 AP / false: 집 공유기에 접속
const char* AP_SSID  = "RD03D-RADAR";
const char* AP_PASS  = "12345678"; // 8자 이상
const char* STA_SSID = "WIFI이름";
const char* STA_PASS = "WIFI비밀번호";

#define TARGET_TIMEOUT_MS 700      // 이 시간 동안 갱신 없으면 표적 소멸

// ---------------------------------------------------------------- RD-03D 명령
// 다중 표적 추적 모드 (최대 3명, x/y/속도 출력)
const uint8_t CMD_MULTI[]  = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x90,0x00,0x04,0x03,0x02,0x01};
// 단일 표적 추적 모드 (가장 강한 1명만, 반응이 더 안정적)
const uint8_t CMD_SINGLE[] = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x80,0x00,0x04,0x03,0x02,0x01};

HardwareSerial Radar(2);
WebServer server(80);

// ---------------------------------------------------------------- 표적 데이터
struct Target {
  bool     active = false;
  float    x = 0, y = 0;     // mm  (x: 좌(-)/우(+),  y: 정면 거리)
  float    speed = 0;        // cm/s
  uint16_t res   = 0;        // 거리분해 파라미터
  uint32_t stamp = 0;
};
Target tg[3];
uint32_t frameCount = 0, lastFrameMs = 0;
bool multiMode = true;

// ---------------------------------------------------------------- 프레임 파서
// RD-03D 다중모드 프레임: AA FF 03 00 | 8byte x 3 표적 | 55 CC   (총 30 byte)
// 표적 8byte: X(2) Y(2) SPEED(2) RES(2)  각 값의 최상위비트=부호(1:양수, 0:음수)
static uint8_t  buf[32];
static uint8_t  idx = 0;

static inline float decodeSigned(uint8_t lo, uint8_t hi) {
  uint16_t mag = ((uint16_t)(hi & 0x7F) << 8) | lo;
  return (hi & 0x80) ? (float)mag : -(float)mag;
}

void parseFrame(const uint8_t* p) {
  uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    const uint8_t* d = p + i * 8;
    float x  = decodeSigned(d[0], d[1]);
    float y  = decodeSigned(d[2], d[3]);
    float sp = decodeSigned(d[4], d[5]);
    uint16_t rs = (uint16_t)d[6] | ((uint16_t)d[7] << 8);

    if (x == 0 && y == 0 && sp == 0) continue;   // 빈 슬롯
    tg[i].x = x; tg[i].y = y; tg[i].speed = sp;
    tg[i].res = rs; tg[i].active = true; tg[i].stamp = now;
  }
  frameCount++; lastFrameMs = now;
}

void readRadar() {
  while (Radar.available()) {
    uint8_t b = Radar.read();
    switch (idx) {
      case 0: if (b == 0xAA) buf[idx++] = b;                     break;
      case 1: if (b == 0xFF) buf[idx++] = b; else idx = (b == 0xAA) ? 1 : 0; break;
      case 2: if (b == 0x03) buf[idx++] = b; else idx = 0;        break;
      case 3: if (b == 0x00) buf[idx++] = b; else idx = 0;        break;
      default:
        buf[idx++] = b;
        if (idx >= 30) {
          if (buf[28] == 0x55 && buf[29] == 0xCC) parseFrame(buf + 4);
          idx = 0;
        }
        break;
    }
  }
  uint32_t now = millis();
  for (int i = 0; i < 3; i++)
    if (tg[i].active && now - tg[i].stamp > TARGET_TIMEOUT_MS) tg[i].active = false;
}

void sendMode(bool multi) {
  multiMode = multi;
  const uint8_t* c = multi ? CMD_MULTI : CMD_SINGLE;
  Radar.write(c, 12);
  Radar.flush();
}

// ---------------------------------------------------------------- 웹 핸들러
void handleData() {
  char out[640];
  int n = snprintf(out, sizeof(out),
      "{\"ms\":%lu,\"mode\":\"%s\",\"frames\":%lu,\"link\":%s,\"targets\":[",
      (unsigned long)millis(), multiMode ? "multi" : "single",
      (unsigned long)frameCount,
      (millis() - lastFrameMs < 1500) ? "true" : "false");

  bool first = true;
  for (int i = 0; i < 3; i++) {
    if (!tg[i].active) continue;
    float xm = tg[i].x / 1000.0f;                 // m
    float ym = tg[i].y / 1000.0f;                 // m
    float dm = sqrtf(xm * xm + ym * ym);          // m
    float ad = atan2f(xm, ym) * 57.29578f;        // deg (0=정면, +=오른쪽)
    n += snprintf(out + n, sizeof(out) - n,
        "%s{\"i\":%d,\"x\":%.2f,\"y\":%.2f,\"d\":%.2f,\"a\":%.1f,\"s\":%.2f}",
        first ? "" : ",", i, xm, ym, dm, ad, tg[i].speed / 100.0f);
    first = false;
  }
  snprintf(out + n, sizeof(out) - n, "]}");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void handleMode() {
  if (server.hasArg("m")) sendMode(server.arg("m") != "single");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", multiMode ? "multi" : "single");
}

// ---------------------------------------------------------------- 시리얼 명령
void printIpInfo() {
  wifi_mode_t m = WiFi.getMode();

  if (m == WIFI_MODE_AP || m == WIFI_MODE_APSTA) {
    Serial.printf("[IP] AP  SSID=%s  IP=%s  접속기기=%d대\n",
                  AP_SSID,
                  WiFi.softAPIP().toString().c_str(),
                  WiFi.softAPgetStationNum());
    Serial.printf("[IP] 접속 주소: http://%s\n", WiFi.softAPIP().toString().c_str());
  }

  if (m == WIFI_MODE_STA || m == WIFI_MODE_APSTA) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[IP] STA SSID=%s  RSSI=%ddBm\n", WiFi.SSID().c_str(), WiFi.RSSI());
      Serial.printf("[IP] IP=%s  GW=%s  MASK=%s\n",
                    WiFi.localIP().toString().c_str(),
                    WiFi.gatewayIP().toString().c_str(),
                    WiFi.subnetMask().toString().c_str());
      Serial.printf("[IP] MAC=%s\n", WiFi.macAddress().c_str());
      Serial.printf("[IP] 접속 주소: http://%s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println(F("[IP] STA 연결 안됨 — 아직 IP 없음"));
    }
  }
}

void runCmd(String c) {
  c.trim(); c.toUpperCase();
  if (c.length() == 0) return;
  if (c == "IP")        printIpInfo();
  else                  Serial.printf("[?] 알 수 없는 명령: %s  (사용 가능: IP)\n", c.c_str());
}

void handleSerialCmd() {
  static char line[32];
  static uint8_t n = 0;
  static uint32_t lastRx = 0;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    lastRx = millis();

    if (ch == '\n' || ch == '\r') {          // 줄바꿈 = 명령 끝
      if (n) { line[n] = 0; runCmd(String(line)); n = 0; }
    } else if (n < sizeof(line) - 1) {
      line[n++] = ch;
    }
  }
  // 줄바꿈 없음(No line ending) 설정에서도 동작하도록
  if (n && millis() - lastRx > 200) { line[n] = 0; runCmd(String(line)); n = 0; }
}

// ---------------------------------------------------------------- 화면 (HTML)
const char PAGE[] PROGMEM = R"PAGE(<!DOCTYPE html>
<html lang="ko"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="theme-color" content="#03100a">
<meta name="mobile-web-app-capable" content="yes">
<title>HEARTBEAT SENSOR</title>
<style>
  :root{
    --ink:#03100a; --phos:#4dff9b; --phos-dim:#1c7a4c;
    --lock:#ff4136; --amber:#ffcf5c; --pane:rgba(6,26,17,.62);
    --side:186px;
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  html,body{margin:0;height:100%;overflow:hidden;background:#000;color:var(--phos);
    font:400 12px/1.45 "DIN Alternate","Menlo","Roboto Mono",ui-monospace,monospace;
    letter-spacing:.06em;-webkit-user-select:none;user-select:none;touch-action:manipulation}
  canvas{position:fixed;left:0;top:0;display:block}
  .pane{position:fixed;background:var(--pane);border:1px solid rgba(77,255,155,.22);
    backdrop-filter:blur(2px);padding:8px 10px}
  #hud{left:10px;top:calc(env(safe-area-inset-top,0px) + 10px);min-width:172px}
  #hud .name{font-size:14px;letter-spacing:.16em;text-shadow:0 0 8px rgba(77,255,155,.6)}
  #hud .sub{color:var(--phos-dim);font-size:10px;letter-spacing:.18em;margin-bottom:6px}
  #hud .row{display:flex;justify-content:space-between;gap:12px;font-size:10.5px}
  #hud .row b{font-weight:400;color:#bdffd8}
  .dot{display:inline-block;width:7px;height:7px;border-radius:50%;margin-right:6px;
    background:var(--lock);box-shadow:0 0 8px currentColor;vertical-align:1px}
  .dot.on{background:var(--phos)}
  #list{right:10px;top:calc(env(safe-area-inset-top,0px) + 10px);width:150px;
    display:flex;flex-direction:column;gap:6px;background:none;border:none;padding:0}
  .card{background:var(--pane);border:1px solid rgba(77,255,155,.22);padding:6px 8px;font-size:10.5px}
  .card.near{border-color:rgba(255,65,54,.75);box-shadow:0 0 14px rgba(255,65,54,.2)}
  .card .id{color:var(--phos-dim);letter-spacing:.2em;font-size:9.5px}
  .card .big{font-size:17px;letter-spacing:.02em;color:#d9ffe9}
  .card .big i{font-size:10px;font-style:normal;color:var(--phos-dim);margin-left:2px}
  .card.near .big{color:#ffd7d4}
  .empty{color:var(--phos-dim);font-size:10px;letter-spacing:.18em;text-align:right;padding-right:2px}
  #bar{left:0;right:0;bottom:0;background:linear-gradient(transparent,rgba(3,16,10,.92) 42%);
    border:none;padding:14px 10px calc(env(safe-area-inset-bottom,0px) + 12px);
    display:flex;gap:8px;justify-content:center;align-items:flex-end;flex-wrap:wrap}
  .grp{display:flex;border:1px solid rgba(77,255,155,.3);overflow:hidden}
  button{appearance:none;background:transparent;border:0;border-right:1px solid rgba(77,255,155,.18);
    color:var(--phos-dim);font:inherit;font-size:11px;letter-spacing:.1em;padding:9px 12px;min-width:44px}
  button:last-child{border-right:0}
  button.on{background:rgba(77,255,155,.16);color:#d9ffe9;text-shadow:0 0 8px rgba(77,255,155,.7)}
  button:focus-visible{outline:2px solid var(--amber);outline-offset:-2px}
  .lbl{color:var(--phos-dim);font-size:9.5px;letter-spacing:.2em;align-self:center;padding:0 2px}

  @media (max-width:410px){
    #hud{min-width:0;padding:6px 8px}
    #hud .row{font-size:9.5px;gap:8px}
    #list{width:112px}
    .card .big{font-size:15px}
    button{padding:9px 9px;min-width:38px;font-size:10.5px}
  }

  /* ===== 가로모드: 레이더 전체화면 + 모든 UI 오른쪽 세로열 ===== */
  body.land #hud{left:auto;right:calc(env(safe-area-inset-right,0px) + 10px);
    top:calc(env(safe-area-inset-top,0px) + 8px);
    width:calc(var(--side) - 18px);min-width:0;padding:6px 9px}
  body.land #hud .row{font-size:9.5px;gap:8px}
  body.land #list{left:auto;right:calc(env(safe-area-inset-right,0px) + 10px);
    top:calc(env(safe-area-inset-top,0px) + 108px);
    width:calc(var(--side) - 18px);max-height:calc(100% - 270px);overflow:hidden}
  body.land #bar{left:auto;top:auto;right:calc(env(safe-area-inset-right,0px) + 10px);
    bottom:calc(env(safe-area-inset-bottom,0px) + 10px);
    width:calc(var(--side) - 18px);background:none;padding:0;
    flex-direction:column;align-items:stretch;justify-content:flex-end;gap:6px;flex-wrap:nowrap}
  body.land #bar .lbl{align-self:flex-start;padding:0}
  body.land #bar .grp{width:100%}
  body.land #bar .grp button{flex:1;min-width:0;padding:8px 2px}

  @media (prefers-reduced-motion:reduce){ .card{transition:none} }
</style></head><body>

<canvas id="cv"></canvas>

<div class="pane" id="hud">
  <div class="name">HEARTBEAT SENSOR</div>
  <div class="sub">HUMAN TRACKING RADAR</div>
  <div class="row"><span><span class="dot" id="dot"></span><span id="stat">연결 중</span></span><b id="cnt">0</b></div>
  <div class="row"><span>갱신</span><b id="rate">0.20 s</b></div>
  <div class="row"><span>탐지범위</span><b id="rng">6.0 m / 120&deg;</b></div>
</div>

<div id="list"></div>

<div class="pane" id="bar">
  <span class="lbl">RANGE</span>
  <div class="grp" id="gr"></div>
  <span class="lbl">MODE</span>
  <div class="grp" id="gm">
    <button data-m="multi" class="on">다중</button><button data-m="single">단일</button>
  </div>
  <div class="grp" id="go">
    <button id="bSnd">&#9834; OFF</button><button id="bFs">&#9974;</button>
  </div>
</div>

<script>
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
const FOV=60, POLL=200;
let W=0,H=0,DPR=1,cx=0,cy=0,R=0,LAND=false;
const S={range:6,sweep:-FOV,dir:1,period:2.4,conn:false,miss:0,sound:false,ac:null};
const T=[0,1,2].map(i=>({i,act:false,seen:false,x:0,y:0,tx:0,ty:0,d:0,pd:0,a:0,s:0,
                         appr:0,glow:0,fade:0,trail:[],tick:0,ping:0}));
const rad=d=>d*Math.PI/180;

/* ---------- 레이아웃: 남은 공간에 레이더를 최대 크기로 ---------- */
const LABEL_PAD=34;   // 방위 눈금 숫자용 좌우 여백
const TOP_PAD=32;     // 0도 눈금 숫자용 상단 여백

function resize(){
  const vv=window.visualViewport;
  W=Math.round(vv?vv.width:innerWidth);
  H=Math.round(vv?vv.height:innerHeight);
  DPR=Math.min(devicePixelRatio||1,2);

  cv.style.width=W+'px'; cv.style.height=H+'px';
  cv.width=Math.round(W*DPR); cv.height=Math.round(H*DPR);
  ctx.setTransform(DPR,0,0,DPR,0,0);

  LAND=(W/H)>=1.30;
  document.body.classList.toggle('land',LAND);

  let x0,x1,y0,y1;
  if(LAND){
    const side=Math.round(Math.min(212,Math.max(148,W*0.155)));
    document.documentElement.style.setProperty('--side',side+'px');
    x0=4; x1=W-side-12; y0=4; y1=H-4;          // 위아래 여백 제거 = 최대 반지름
  }else{
    document.documentElement.style.setProperty('--side','186px');
    x0=6; x1=W-6; y0=108; y1=H-116;            // HUD / 조작바 자리만 확보
  }

  const aw=Math.max(140,x1-x0), ah=Math.max(140,y1-y0);
  R=Math.max(70,Math.min((aw/2-LABEL_PAD)/Math.sin(rad(FOV)), ah-TOP_PAD));
  cx=(x0+x1)/2;
  cy=y1-Math.max(0,(ah-(R+TOP_PAD))/2);        // 남는 높이는 위아래 균등 분배
}
addEventListener('resize',resize);
addEventListener('orientationchange',()=>{setTimeout(resize,150);setTimeout(resize,450);});
if(window.visualViewport){
  visualViewport.addEventListener('resize',resize);
  visualViewport.addEventListener('scroll',resize);
}
resize();

/* ---------- 전체화면 ---------- */
const bFs=document.getElementById('bFs');
function isFs(){return !!(document.fullscreenElement||document.webkitFullscreenElement);}
async function toggleFs(){
  try{
    if(!isFs()){
      const el=document.documentElement;
      if(el.requestFullscreen) await el.requestFullscreen({navigationUI:'hide'});
      else if(el.webkitRequestFullscreen) el.webkitRequestFullscreen();
      if(screen.orientation&&screen.orientation.lock)
        screen.orientation.lock('landscape').catch(()=>{});
    }else{
      if(screen.orientation&&screen.orientation.unlock){try{screen.orientation.unlock();}catch(e){}}
      if(document.exitFullscreen) await document.exitFullscreen();
      else if(document.webkitExitFullscreen) document.webkitExitFullscreen();
    }
  }catch(e){}
  setTimeout(resize,120); setTimeout(resize,420);
}
bFs.onclick=toggleFs;
function fsChanged(){ bFs.className=isFs()?'on':''; setTimeout(resize,120); }
document.addEventListener('fullscreenchange',fsChanged);
document.addEventListener('webkitfullscreenchange',fsChanged);
/* 레이더 화면 두 번 탭 -> 전체화면 토글 */
let tapT=0;
cv.addEventListener('pointerup',()=>{const n=Date.now(); if(n-tapT<320){toggleFs();tapT=0;} else tapT=n;});

function pos(x,y){return[cx+(x/S.range)*R, cy-(y/S.range)*R];}
function sector(r){ctx.beginPath();ctx.moveTo(cx,cy);ctx.arc(cx,cy,r,rad(-90-FOV),rad(-90+FOV));ctx.closePath();}

/* ---------- 데이터 수신 : 0.2초 주기 ---------- */
function apply(j){
  const inc=new Map((j.targets||[]).map(t=>[t.i,t]));
  T.forEach(t=>{
    const n=inc.get(t.i);
    if(n){
      if(!t.act){t.x=n.x;t.y=n.y;t.trail.length=0;}
      t.tx=n.x; t.ty=n.y; t.pd=t.d||n.d; t.d=n.d; t.a=n.a; t.s=n.s;
      t.appr=t.appr*0.6+(t.pd-t.d)*0.4;
      t.act=true; t.fade=1;
    } else t.act=false;
  });
  document.getElementById('cnt').textContent=(j.targets||[]).length;
}
async function poll(){
  try{
    const c=new AbortController(), to=setTimeout(()=>c.abort(),450);
    const r=await fetch('/data?_='+Date.now(),{cache:'no-store',signal:c.signal});
    clearTimeout(to); apply(await r.json()); S.conn=true; S.miss=0;
  }catch(e){ if(++S.miss>2){S.conn=false; T.forEach(t=>t.act=false);} }
}
setInterval(poll,POLL); poll();

/* ---------- 소리 ---------- */
function ping(f){
  if(!S.sound||!S.ac)return;
  const o=S.ac.createOscillator(), g=S.ac.createGain(), t=S.ac.currentTime;
  o.type='sine'; o.frequency.value=f;
  g.gain.setValueAtTime(0,t); g.gain.linearRampToValueAtTime(.09,t+.01);
  g.gain.exponentialRampToValueAtTime(.0001,t+.22);
  o.connect(g).connect(S.ac.destination); o.start(t); o.stop(t+.24);
}

/* ---------- 그리기 ---------- */
function grid(){
  const bg=ctx.createRadialGradient(cx,cy,0,cx,cy,R);
  bg.addColorStop(0,'rgba(24,92,60,.42)'); bg.addColorStop(.72,'rgba(8,38,25,.30)');
  bg.addColorStop(1,'rgba(3,16,10,.12)');
  sector(R); ctx.fillStyle=bg; ctx.fill();

  const cell=Math.max(22,Math.round(R/12));
  ctx.save(); sector(R); ctx.clip();
  ctx.strokeStyle='rgba(77,255,155,.055)'; ctx.lineWidth=1; ctx.beginPath();
  for(let x=cx%cell;x<W;x+=cell){ctx.moveTo(x,cy-R);ctx.lineTo(x,cy);}
  for(let y=cy;y>cy-R;y-=cell){ctx.moveTo(cx-R,y);ctx.lineTo(cx+R,y);}
  ctx.stroke();

  for(let m=1;m<=S.range;m++){
    const r=R*m/S.range, last=(m===S.range);
    ctx.beginPath(); ctx.arc(cx,cy,r,rad(-90-FOV),rad(-90+FOV));
    ctx.setLineDash(last?[]:[7,7]); ctx.lineWidth=last?1.4:1;
    ctx.strokeStyle=last?'rgba(77,255,155,.45)':'rgba(210,255,230,.30)'; ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle='rgba(150,235,190,.55)'; ctx.font='9px monospace'; ctx.textAlign='left';
    ctx.fillText(m+'m',cx+4,cy-r-4);
  }
  ctx.beginPath();
  for(let a=-FOV;a<=FOV;a+=10){
    const e=pos(S.range*Math.sin(rad(a)),S.range*Math.cos(rad(a)));
    ctx.moveTo(cx,cy); ctx.lineTo(e[0],e[1]);
  }
  ctx.strokeStyle='rgba(77,255,155,.10)'; ctx.lineWidth=1; ctx.stroke();
  ctx.restore();
}

function scale(){
  ctx.save(); ctx.lineWidth=1;
  ctx.beginPath(); ctx.arc(cx,cy,R,rad(-90-FOV),rad(-90+FOV));
  ctx.strokeStyle='rgba(210,255,230,.55)'; ctx.stroke();
  for(let a=-FOV;a<=FOV;a+=2.5){
    const maj=Math.abs(a%10)<.01, mid=Math.abs(a%5)<.01;
    const L=maj?9:(mid?6:3.5), s=R+1, e=R+1+L;
    const u=Math.sin(rad(a)), v=-Math.cos(rad(a));
    ctx.beginPath(); ctx.moveTo(cx+u*s,cy+v*s); ctx.lineTo(cx+u*e,cy+v*e);
    ctx.strokeStyle=maj?'rgba(230,255,240,.8)':'rgba(180,240,205,.4)';
    ctx.lineWidth=maj?1.4:1; ctx.stroke();
    if(maj){
      const lb=a<0?(360+a):a, r2=R+22;
      ctx.save(); ctx.translate(cx+u*r2,cy+v*r2); ctx.rotate(rad(a));
      ctx.fillStyle='rgba(220,255,235,.72)'; ctx.font='10px monospace';
      ctx.textAlign='center'; ctx.textBaseline='middle'; ctx.fillText(lb,0,0); ctx.restore();
    }
  }
  ctx.restore();
}

function sweepBeam(){
  ctx.save(); sector(R); ctx.clip();
  const TR=52,N=26;
  for(let k=0;k<N;k++){
    const a0=S.sweep-S.dir*TR*k/N, a1=S.sweep-S.dir*TR*(k+1)/N;
    ctx.beginPath(); ctx.moveTo(cx,cy);
    ctx.arc(cx,cy,R,rad(-90+Math.min(a0,a1))-0.002,rad(-90+Math.max(a0,a1))+0.002);
    ctx.closePath();
    ctx.fillStyle='rgba(96,255,160,'+(0.15*Math.pow(1-k/N,2.1)).toFixed(4)+')'; ctx.fill();
  }
  const e=pos(S.range*Math.sin(rad(S.sweep)),S.range*Math.cos(rad(S.sweep)));
  const g=ctx.createLinearGradient(cx,cy,e[0],e[1]);
  g.addColorStop(0,'rgba(180,255,215,.10)'); g.addColorStop(.55,'rgba(190,255,220,.75)');
  g.addColorStop(1,'rgba(220,255,235,.95)');
  ctx.strokeStyle=g; ctx.lineWidth=2.6; ctx.shadowColor='rgba(120,255,180,.9)'; ctx.shadowBlur=16;
  ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(e[0],e[1]); ctx.stroke();
  ctx.restore();
}

function blip(t,near){
  const [px,py]=pos(t.x,t.y), rr=9+5*t.glow, al=t.fade;
  ctx.save();
  t.trail.forEach((p,i)=>{
    const [qx,qy]=pos(p.x,p.y), a=(1-p.age/1.5)*.32*al;
    if(a<=0)return;
    ctx.beginPath(); ctx.arc(qx,qy,2.2+2*(1-p.age/1.5),0,6.283);
    ctx.fillStyle='rgba(77,255,155,'+a.toFixed(3)+')'; ctx.fill();
  });
  const g=ctx.createRadialGradient(px,py,0,px,py,rr*2.6);
  g.addColorStop(0,'rgba(235,255,240,'+(0.95*al)+')');
  g.addColorStop(.28,'rgba(110,255,170,'+(0.85*al)+')');
  g.addColorStop(1,'rgba(60,220,130,0)');
  ctx.beginPath(); ctx.arc(px,py,rr*2.6,0,6.283); ctx.fillStyle=g; ctx.fill();
  ctx.beginPath(); ctx.arc(px,py,rr,0,6.283);
  ctx.strokeStyle='rgba(215,255,230,'+(0.55*al)+')'; ctx.lineWidth=1; ctx.stroke();

  if(near){
    const b=15, c='rgba(255,65,54,'+(0.95*al)+')';
    ctx.strokeStyle=c; ctx.lineWidth=1.6;
    [[-1,-1],[1,-1],[-1,1],[1,1]].forEach(([sx,sy])=>{
      ctx.beginPath();
      ctx.moveTo(px+sx*b,py+sy*b-sy*6); ctx.lineTo(px+sx*b,py+sy*b);
      ctx.lineTo(px+sx*b-sx*6,py+sy*b); ctx.stroke();
    });
    ctx.fillStyle=c; ctx.font='10px monospace'; ctx.textAlign='center';
    ctx.fillText('LOCK',px,py-b-7);
  }
  ctx.fillStyle='rgba(225,255,238,'+(0.9*al)+')'; ctx.font='11px monospace'; ctx.textAlign='left';
  ctx.fillText(t.d.toFixed(2)+'m',px+18,py+4);
  ctx.restore();
}

function crt(){
  ctx.save(); ctx.globalCompositeOperation='overlay'; ctx.fillStyle='rgba(0,0,0,.16)';
  for(let y=0;y<H;y+=3) ctx.fillRect(0,y,W,1);
  ctx.restore();
  const v=ctx.createRadialGradient(cx,cy-R*.35,R*.55,cx,cy-R*.35,R*1.5);
  v.addColorStop(0,'rgba(0,0,0,0)'); v.addColorStop(1,'rgba(0,0,0,.55)');
  ctx.fillStyle=v; ctx.fillRect(0,0,W,H);
}

/* ---------- 메인 루프 (60fps) ---------- */
let last=performance.now();
function frame(now){
  const dt=Math.min((now-last)/1000,.05); last=now;

  const prev=S.sweep;
  S.sweep+=S.dir*(2*FOV/S.period)*dt;
  if(S.sweep>FOV){S.sweep=FOV;S.dir=-1;} if(S.sweep<-FOV){S.sweep=-FOV;S.dir=1;}

  const k=1-Math.exp(-dt/0.09);
  T.forEach(t=>{
    t.x+=(t.tx-t.x)*k; t.y+=(t.ty-t.y)*k;
    t.fade=t.act?Math.min(1,t.fade+dt*8):Math.max(0,t.fade-dt*2.2);
    t.glow=Math.max(0,t.glow-dt*1.8);
    if(t.act){
      t.tick+=dt;
      if(t.tick>0.07){t.trail.unshift({x:t.x,y:t.y,age:0});t.tick=0;}
      t.trail.forEach(p=>p.age+=dt);
      if(t.trail.length>40)t.trail.length=40;
      t.trail=t.trail.filter(p=>p.age<1.5);
      const a=t.a;
      if((prev-a)*(S.sweep-a)<=0){ t.glow=1; if(now-t.ping>300){ping(880-t.d*60);t.ping=now;} }
    } else if(t.trail.length) t.trail.forEach(p=>p.age+=dt*2);
  });

  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#03100a'; ctx.fillRect(0,0,W,H);
  grid(); sweepBeam();
  const live=T.filter(t=>t.fade>0.02);
  const near=live.filter(t=>t.act).sort((a,b)=>a.d-b.d)[0];
  live.forEach(t=>blip(t,t===near));
  scale(); crt();
  hud(near);
  requestAnimationFrame(frame);
}

function hud(near){
  const d=document.getElementById('dot'), s=document.getElementById('stat');
  d.className='dot'+(S.conn?' on':''); s.textContent=S.conn?'수신 중':'신호 없음';
  document.getElementById('rng').innerHTML=S.range.toFixed(1)+' m / 120&deg;';
  const box=document.getElementById('list');
  const act=T.filter(t=>t.act);
  if(!act.length){box.innerHTML='<div class="empty">표적 없음</div>';return;}
  box.innerHTML=act.map(t=>{
    const dir=t.appr>0.02?'▲ 접근':(t.appr<-0.02?'▼ 이탈':'— 정지');
    const brg=(t.a<0?'L':'R')+Math.abs(t.a).toFixed(0)+'\u00B0';
    return '<div class="card'+(t===near?' near':'')+'">'
      +'<div class="id">TGT '+(t.i+1)+' &nbsp;'+brg+'</div>'
      +'<div class="big">'+t.d.toFixed(2)+'<i>m</i></div>'
      +'<div class="id">'+dir+' &nbsp;'+Math.abs(t.s).toFixed(1)+'m/s</div></div>';
  }).join('');
}
requestAnimationFrame(frame);

/* ---------- 조작 ---------- */
const gr=document.getElementById('gr');
[2,4,6,8].forEach(v=>{
  const b=document.createElement('button'); b.textContent=v+'m'; if(v===S.range)b.className='on';
  b.onclick=()=>{S.range=v;[...gr.children].forEach(c=>c.className='');b.className='on';};
  gr.appendChild(b);
});
document.getElementById('gm').onclick=e=>{
  const b=e.target.closest('button'); if(!b)return;
  [...b.parentNode.children].forEach(c=>c.className=''); b.className='on';
  fetch('/mode?m='+b.dataset.m).catch(()=>{});
};
const bs=document.getElementById('bSnd');
bs.onclick=()=>{
  S.sound=!S.sound; bs.className=S.sound?'on':''; bs.textContent='\u266A '+(S.sound?'ON':'OFF');
  if(S.sound&&!S.ac) S.ac=new (window.AudioContext||window.webkitAudioContext)();
  if(S.ac) S.ac.resume();
};
</script></body></html>
)PAGE";

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);

  Radar.setRxBufferSize(2048);
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(200);
  sendMode(true);                       // 다중 표적 모드로 시작
  Serial.println(F("\n[RD-03D] UART 256000 시작"));

  if (USE_AP) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print(F("[WiFi] AP: ")); Serial.println(AP_SSID);
    Serial.print(F("[WiFi] 접속 주소: http://")); Serial.println(WiFi.softAPIP());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.print(F("[WiFi] 연결 중"));
    while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print('.'); }
    Serial.print(F("\n[WiFi] 접속 주소: http://")); Serial.println(WiFi.localIP());
  }

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "max-age=600");
    server.send_P(200, "text/html", PAGE);
  });
  server.on("/data", HTTP_GET, handleData);
  server.on("/mode", HTTP_GET, handleMode);
  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();
}

void loop() {
  readRadar();
  server.handleClient();
  handleSerialCmd();

  static uint32_t t = 0;
  if (millis() - t > 1000) {
    t = millis();
    IPAddress ip = (WiFi.getMode() == WIFI_MODE_STA) ? WiFi.localIP() : WiFi.softAPIP();
    Serial.printf("frames=%lu  T1:%d T2:%d T3:%d  http://%s\n",
                  (unsigned long)frameCount, tg[0].active, tg[1].active, tg[2].active,
                  ip.toString().c_str());
  }
}
