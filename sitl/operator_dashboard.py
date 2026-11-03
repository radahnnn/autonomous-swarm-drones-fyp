"""Small, readable browser dashboard for the independent swarm operator."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import re
from pathlib import Path
import math
import threading
import time

from sitl.formation_library import STYLES


STYLE_LABELS = {
    "v": "V", "line": "Line", "column": "Column", "diamond": "Diamond",
    "ring": "Ring", "echelon": "Echelon", "triangle": "Triangle",
    "pairs": "Pairs A", "pairs_b": "Pairs B",
}


def _minimum_separation(agents):
    positions = []
    for agent in agents:
        position = agent.get("position")
        if (isinstance(position, (list, tuple)) and len(position) >= 2 and
                all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in position[:2])):
            positions.append(position[:2])
    if len(positions) < 2:
        return None
    return min(math.hypot(a[0] - b[0], a[1] - b[1])
               for index, a in enumerate(positions)
               for b in positions[index + 1:])


PAGE = r'''<!doctype html><html><head><meta charset="utf-8"><title>Swarm Control</title>
<style>
:root{--bg:#0c111b;--card:#151d2b;--edge:#28364b;--text:#eef4ff;--muted:#91a0b8;--ok:#38d996;--warn:#ffbd59;--bad:#ff6474;--blue:#62a8ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,sans-serif;height:100vh;display:grid;grid-template-columns:minmax(0,1fr) 360px}
main{position:relative;min-width:0}canvas{width:100%;height:100%;display:block}.top{position:absolute;top:18px;left:18px;right:18px;display:flex;gap:10px;pointer-events:none}.pill{background:#111a28e8;border:1px solid var(--edge);border-radius:12px;padding:10px 14px;box-shadow:0 8px 30px #0005}.pill b{font-size:18px}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
aside{background:var(--card);border-left:1px solid var(--edge);padding:20px;overflow:auto}h1{font-size:20px;margin:0 0 4px}h2{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.11em;margin:22px 0 9px}.sub{color:var(--muted);font-size:13px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}.grid.two{grid-template-columns:1fr 1fr}button{border:1px solid var(--edge);background:#202c3e;color:var(--text);padding:11px 8px;border-radius:9px;font-weight:650;cursor:pointer}button:hover{background:#2a3a51}button:disabled{opacity:.42;cursor:not-allowed}.modebtn.active,.shape.active{border-color:var(--blue);background:#17375d;box-shadow:0 0 0 1px var(--blue) inset}.launch{background:#176c4b}.land{background:#762c38}.agentsgrid{display:grid;grid-template-columns:1fr 1fr;gap:7px}.card{background:#101824;border:1px solid var(--edge);border-radius:10px;padding:9px;margin:0;min-width:0}.row{display:flex;justify-content:space-between;gap:6px}.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:6px;background:var(--ok)}input{width:100%;background:#0d1420;color:var(--text);border:1px solid var(--edge);border-radius:8px;padding:9px}.fault{display:grid;grid-template-columns:1fr 1fr;gap:7px}.msg{min-height:36px;color:var(--muted);font-size:13px;margin-top:10px}details{margin-top:18px;color:var(--muted)}pre{white-space:pre-wrap;font-size:11px}
.bar{position:relative;height:10px;background:#1c293b;border-radius:6px;margin:8px 0;overflow:hidden}.bar i{display:block;height:100%;width:0;background:var(--ok);transition:width .3s}.bar u{position:absolute;top:0;bottom:0;width:2px;background:var(--bad)}.mx{font-variant-numeric:tabular-nums}#rp{position:absolute;left:18px;right:18px;bottom:16px;background:#111a28f0;border:1px solid var(--edge);border-radius:12px;padding:10px 14px;display:none;gap:10px;align-items:center;flex-wrap:wrap}#rp input[type=range]{flex:1;min-width:180px}#rp button,#rp select{background:#1c293b;color:var(--text);border:1px solid var(--edge);border-radius:8px;padding:6px 10px}.big{font-size:22px}
</style></head><body><main><canvas id="map"></canvas><div id="rp"><button id="rpplay" onclick="rpPlay()">⏸</button><button onclick="rpJump(-1)" title="previous event">⏮</button><input id="rpseek" type="range" min="0" max="1000" value="0" oninput="rpDrag=true" onchange="rpSeek(this.value)"><button onclick="rpJump(1)" title="next event">⏭</button><select id="rpspeed" onchange="rpSpeed(this.value)"><option>0.5</option><option>1</option><option>2</option><option selected>4</option><option>8</option><option>16</option></select>×<b id="rptime" class="mx"></b><span class="sub" id="rpmark"></span></div><div class="top"><div class="pill"><b id="mode">CONNECTING</b><br><span id="mission" class="sub">Waiting for status</span></div><div class="pill"><b id="health">0 / 0</b><br><span class="sub">healthy agents</span></div><div class="pill"><b id="leader">—</b><br><span class="sub">elected leader</span></div></div></main>
<aside><h1>Swarm Control</h1><div class="sub">Simple operator view · map shows local North/East</div>
<h2>Control architecture</h2><div class="grid"><button class="modebtn" data-mode="centralized" onclick="chooseMode('centralized')">Centralized</button><button class="modebtn" data-mode="decentralized" onclick="chooseMode('decentralized')">Decentralized</button><button class="modebtn" data-mode="hybrid" onclick="chooseMode('hybrid')">Hybrid</button></div><div id="modehelp" class="sub" style="margin-top:7px">Loading active architecture…</div>
<h2>Flight</h2><div class="grid"><button onclick="send('preflight')">Check safety</button><button class="launch" onclick="send('launch')">Launch all</button><button class="land" onclick="land()">Land all</button></div>
<h2>Move swarm</h2><div class="grid"><span></span><button onclick="send('w 5')">↑ North</button><span></span><button onclick="send('a 5')">← West</button><button onclick="send('status')">Refresh</button><button onclick="send('d 5')">East →</button><span></span><button onclick="send('s 5')">↓ South</button><span></span></div>
<h2>Formation shape</h2><div class="grid" id="shapes"></div><div class="sub" id="shapehelp">Shape changes are available grounded, or in flight after safety checks.</div>
<h2>Altitude</h2><div class="grid two"><button onclick="send('u 2')">Up 2 m</button><button onclick="send('j 2')">Down 2 m</button></div>
<h2>Mission</h2><div class="grid"><button onclick="send('patrol square 12')">Start patrol</button><button onclick="send('route stop')">Stop patrol</button><button onclick="send('goto 0 0')">Return home</button><button onclick="send('capture dashboard')">Capture images</button><button onclick="send('coord off')">Cut coordinator</button><button onclick="send('coord on')">Restore coordinator</button></div><div class="sub" id="realism" style="margin-top:7px"></div>
<h2>Camera</h2><div class="sub" id="caphelp">No images yet (Gazebo world with cameras, then "Capture images").</div><div id="gallery" style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px"></div>
<h2>Communication test</h2><div class="fault"><input id="from" type="number" min="1" value="1" aria-label="sender"><input id="to" type="number" min="1" value="2" aria-label="recipient"><button id="blocklink" onclick="link(true)">Block link</button><button id="restorelink" onclick="link(false)">Restore link</button></div><div class="sub" style="margin-top:7px">Cuts one direction only. The remaining drones should keep flying.</div>
<h2>Run metrics</h2><div class="card mx" id="mx"><div class="row"><b class="big" id="mxgap">—</b><span class="sub" id="mxstate">waiting for flight</span></div><div class="bar"><i id="mxbar"></i><u style="left:50%" title="separation gate"></u></div><canvas id="mxspark" style="height:56px;width:100%"></canvas><div class="sub" id="mxline"></div><div class="sub" id="mxdrones"></div><div class="sub" id="mxcount"></div><div class="sub" id="mxcoord"></div></div>
<h2>Drone health & recovery</h2><div class="sub" id="recoveryhelp">Recovery controls depend on the active architecture.</div><div id="agents" class="agentsgrid"></div><div class="msg" id="msg"></div>
<details><summary>Technical details</summary><pre id="raw"></pre></details></aside>
<script>
let state=null,trail={};const c=document.getElementById('map'),x=c.getContext('2d');
function resize(){c.width=c.clientWidth*devicePixelRatio;c.height=c.clientHeight*devicePixelRatio}addEventListener('resize',resize);resize();
async function send(command){msg.textContent='Sending: '+command;try{let r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command})});let j=await r.json();let p=j.preflight;if(p){let gap=Number.isFinite(p.minimum_separation_m)?' Minimum separation: '+p.minimum_separation_m.toFixed(2)+' m.':'';let reasons=[...new Set((p.agents||[]).flatMap(a=>a.reasons||[]))];msg.textContent=j.message+gap+(reasons.length?' Reasons: '+reasons.join(', ')+'.':'')}else msg.textContent=j.message}catch(e){msg.textContent='Command failed: '+e}}
function land(){if(confirm('Land all drones now?'))send('land')}function link(block){let a=from.value,b=to.value;send((block?'link-block ':'link-restore ')+a+' '+b)}
async function setShape(style){if(!state||!state.formation_change_allowed||state.formation_transitioning){msg.textContent='Wait for fresh safe status and for any current formation transition to finish.';return}msg.textContent='Planning safe transition to '+style+'…';try{let r=await fetch('/formation',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({formation:style})});let j=await r.json();msg.textContent=j.message}catch(e){msg.textContent='Formation change failed: '+e}}
async function agentAction(id,online){let centralized=state&&state.control_mode==='centralized';let detail=online?(centralized?'Fail vehicle D'+id+'? The centralized console will command LAND for that SITL vehicle.':'Stop D'+id+' software agent? Its SITL flight controller stays active and may hold its last target if airborne.'):(centralized?'Revive vehicle D'+id+' in the swarm?':'Restart D'+id+' software agent? If airborne, the replacement agent will command LAND.');if(!confirm(detail))return;send((online?'fail ':'restart ')+id)}
async function chooseMode(choice){if(!state)return;let flying=state.airborne||state.armed;if(flying&&!state.mode_switch_enabled){msg.textContent='Land and disarm all drones before changing architecture.';return}if(choice===state.control_mode){msg.textContent=choice+' is already active.';return}if(!confirm(flying?('Switch all drones to '+choice+' mode in flight? It is synchronized and blended over a few seconds, and rolls back automatically if any drone fails to follow.'):('Switch to '+choice+' mode? The controller will restart after confirming all drones are grounded and disarmed; SITL and QGroundControl will stay running.')))return;try{let r=await fetch('/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:choice})});let j=await r.json();msg.textContent=j.message}catch(e){msg.textContent='Mode switch failed: '+e}}
let rpDrag=false;async function rpPost(b){try{await fetch('/replay',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})}catch(e){}}
function rpPlay(){rpPost({action:state&&state.replay&&state.replay.playing?'pause':'play'})}function rpSpeed(v){rpPost({action:'speed',speed:+v})}
function rpSeek(v){rpDrag=false;if(state&&state.replay)rpPost({action:'seek',t:v/1000*state.replay.duration})}
function rpJump(d){let r=state&&state.replay;if(!r)return;let m=r.markers.map(k=>k.t).sort((a,b)=>a-b),t=d>0?m.find(v=>v>r.t+0.6):[...m].reverse().find(v=>v<r.t-0.6);if(t!==undefined)rpPost({action:'seek',t:Math.max(0,t-1)})}
function renderReplay(){let r=state&&state.replay,bar=document.getElementById('rp');if(!r){bar.style.display='none';return}bar.style.display='flex';document.getElementById('rpplay').textContent=r.playing?'⏸':'▶';
if(!rpDrag)document.getElementById('rpseek').value=Math.round(r.t/Math.max(0.01,r.duration)*1000);document.getElementById('rptime').textContent=r.t.toFixed(0)+' / '+r.duration.toFixed(0)+' s';
let past=r.markers.filter(k=>k.t<=r.t).slice(-1)[0];document.getElementById('rpmark').textContent=past?'last event: '+past.label+' (t='+past.t.toFixed(0)+'s)':'no event yet';}
function renderMetrics(){let m=state&&state.metrics,g=document.getElementById('mxgap');if(!m){document.getElementById('mx').style.display='none';return}document.getElementById('mx').style.display='';
let col=m.state==='violation'?'var(--bad)':m.state==='warn'?'var(--warn)':'var(--ok)';g.textContent=m.gap_m==null?'—':m.gap_m.toFixed(2)+' m';g.style.color=col;
document.getElementById('mxstate').textContent=m.gap_m==null?'waiting for flight':(m.state==='ok'?'OK':m.state==='warn'?'LOW MARGIN':'BELOW GATE')+' · gate '+m.gate_m+' m · margin '+(m.margin_m>=0?'+':'')+m.margin_m.toFixed(2)+' m'+(m.pair?' · D'+m.pair[0]+'–D'+m.pair[1]:'');
let b=document.getElementById('mxbar');b.style.width=m.gap_m==null?'0%':Math.max(2,Math.min(100,m.gap_m/(2*m.gate_m)*100))+'%';b.style.background=col;
let cv=document.getElementById('mxspark'),w=cv.clientWidth*devicePixelRatio,h=cv.clientHeight*devicePixelRatio;if(cv.width!==w)cv.width=w;if(cv.height!==h)cv.height=h;let q=cv.getContext('2d');q.clearRect(0,0,w,h);let hist=m.history||[];
if(hist.length>1){let t0=hist[0][0],t1=hist[hist.length-1][0]||1,top=m.gate_m*3,sx=t=>(t-t0)/Math.max(1,t1-t0)*w,sy=v=>h-Math.min(v,top)/top*h;q.strokeStyle='#ff6474';q.setLineDash([4,4]);q.beginPath();q.moveTo(0,sy(m.gate_m));q.lineTo(w,sy(m.gate_m));q.stroke();q.setLineDash([]);q.strokeStyle='#62a8ff';q.lineWidth=1.5*devicePixelRatio;q.beginPath();hist.forEach((p,i)=>i?q.lineTo(sx(p[0]),sy(p[1])):q.moveTo(sx(p[0]),sy(p[1])));q.stroke()}
document.getElementById('mxline').textContent=(m.min_gap_m==null?'':'lowest '+m.min_gap_m.toFixed(2)+' m · ')+(m.gap_3d_m==null?'':'3-D gap now '+m.gap_3d_m.toFixed(2)+' m · ')+'violations '+m.violations+(m.violations?' ('+m.violation_time_s+' s)':'')+' · low-margin episodes '+m.warnings+' · sampled ~1 Hz';
let modes={},err=[],age=0;(m.drones||[]).forEach(d=>{if(d.mode)modes[d.mode]=(modes[d.mode]||0)+1;if(d.tracking_error_m!=null)err.push(d.tracking_error_m);if(d.link_age_s!=null)age=Math.max(age,d.link_age_s)});
document.getElementById('mxdrones').textContent='modes '+(Object.entries(modes).map(([k,v])=>v+'× '+k).join(', ')||'—')+' · tracking err '+(err.length?(err.reduce((a,c)=>a+c,0)/err.length).toFixed(2)+' m avg / '+Math.max(...err).toFixed(2)+' max':'—')+' · oldest status '+age.toFixed(1)+' s';
let co=state.coordinator||{},ct=document.getElementById('mxcoord');ct.textContent=co.source?('coordinator: '+(co.acting&&co.acting.length?'ACTING D'+co.acting.join(',D')+' (term '+co.term+')':co.source)):'';ct.style.color=co.acting&&co.acting.length?'var(--warn)':'';
let c=m.counters||{};document.getElementById('mxcount').textContent=Object.keys(c).length?Object.entries(c).map(([k,v])=>k.replace(/_/g,' ')+' '+v).join(' · '):''}
async function poll(){try{let r=await fetch('/state',{cache:'no-store'});state=await r.json();update()}catch(e){mode.textContent='DISCONNECTED';mode.className='bad'}setTimeout(poll,350)}
function renderExtras(){let r=state.realism||{};document.getElementById('realism').textContent=r.profile&&r.profile!=='matek-single-compass'&&r.profile!=='baseline'?('Simulation: '+r.profile+' · GPS σ '+r.position_sigma_m+' m · spacing ×'+r.formation_scale):'Simulation: ideal (no disturbances)';let c=state.captures,g=document.getElementById('gallery'),h=document.getElementById('caphelp');if(c&&c.frames&&c.frames.length){h.textContent='Latest capture: '+c.label+' · '+(c.utc||'').slice(11,19)+' UTC';let html=c.frames.map(f=>'<a href="/captures/'+f.file+'" target="_blank"><img src="/captures/'+f.file+'" title="D'+f.drone+'" style="width:100%;border-radius:4px"></a>').join('');if(g.dataset.k!==c.label+c.utc){g.innerHTML=html;g.dataset.k=c.label+c.utc}}}
function update(){renderExtras();renderMetrics();renderReplay();mode.textContent=state.transitioning?'SWITCHING…':state.control_mode.toUpperCase();mode.className=state.transitioning?'warn':(state.healthy===state.count?'ok':'warn');health.textContent=state.healthy+' / '+state.count;leader.textContent=state.leader==null?'—':'D'+state.leader;let gap=Number.isFinite(state.minimum_separation_m)?' · min gap '+state.minimum_separation_m.toFixed(1)+' m':'';mission.textContent=state.airborne+' airborne · '+state.formation.toUpperCase()+gap+' · target N '+state.goal[0].toFixed(1)+' E '+state.goal[1].toFixed(1)+' · '+state.altitude.toFixed(1)+' m';
 document.querySelectorAll('.modebtn').forEach(b=>{let active=b.dataset.mode===state.control_mode;b.classList.toggle('active',active);b.disabled=!!state.transitioning||!!state.mode_switching||((state.airborne>0||state.armed>0)&&!state.mode_switch_enabled)||!state.healthy});modehelp.textContent=state.mode_switching?('Switching to '+state.mode_switch_target+' ('+state.mode_switch_state+')…'):(state.transitioning?'Switching safely to '+state.switch_target+'; the dashboard stays connected.':(state.transition_error||(state.mode_switch_enabled?'Click another mode to switch all drones in flight (synchronized, blended, rolls back on failure).':'Select another mode to restart the grounded controller while SITL remains running.')));
 shapes.innerHTML=state.formations.map(style=>'<button class="shape" onclick="setShape(\''+style+'\')">'+state.formation_labels[style]+'</button>').join('');document.querySelectorAll('.shape').forEach(b=>{b.classList.toggle('active',b.textContent.toLowerCase()===state.formation);b.disabled=!state.formation_change_enabled||!state.formation_change_allowed||!!state.formation_transitioning||!!state.transitioning});shapehelp.textContent=!state.formation_change_enabled?'Formation controls are unavailable in this mode.':(state.formation_transitioning?'Smooth formation transition: '+Math.round(100*(state.formation_transition_progress||0))+'% complete; wait before issuing another change.':(!state.formation_change_allowed?'Waiting for fresh, healthy status and a safe all-grounded or all-airborne swarm.':'Every requested shape is screened for path clearance; unsafe or crowded transitions are rejected.'));
 document.getElementById('blocklink').disabled=!state.peer_fault_enabled||!!state.transitioning;document.getElementById('restorelink').disabled=!state.peer_fault_enabled||!!state.transitioning;recoveryhelp.textContent=state.recovery_help||'Recovery controls depend on the active architecture.';
 agents.innerHTML=state.agents.map(a=>'<div class="card"><div class="row"><b><span class="dot" style="background:'+(a.health==='OK'?'var(--ok)':a.health==='STALE'?'var(--bad)':'var(--warn)')+'"></span>D'+a.id+(a.leader?' ★':'')+'</b><b>'+a.health+'</b></div><div class="row sub"><span>'+(a.airborne?'Airborne':'Grounded')+' · '+a.safety+'</span><span>'+a.members+' members</span></div><div class="row sub"><span>Agent '+(a.agent_online?'online':'stopped')+'</span><span>'+String(a.formation||state.formation).toUpperCase()+'</span></div><div class="row sub"><span style="color:'+(a.estimate==='ok'?'inherit':'var(--bad)')+'">GPS '+(a.estimate==='ok'?'ok':a.estimate.toUpperCase())+'</span><span>'+(a.filter_active?'⛨ avoiding':'')+(a.obstacles_known?' · '+a.obstacles_known+' obst.':'')+'</span></div><button onclick="agentAction('+a.id+','+(a.agent_online?'true':'false')+')" '+(a.agent_action_enabled?'':'disabled')+'>'+(a.agent_online?(state.recovery_label||'Kill agent'):'Revive / restart')+'</button></div>').join('');raw.textContent=JSON.stringify(state,null,2)}
function draw(){requestAnimationFrame(draw);x.clearRect(0,0,c.width,c.height);let s=18*devicePixelRatio,W=c.width,H=c.height,cn=0,ce=0,n=0;if(state)state.agents.forEach(a=>{if(a.position){cn+=a.position[0];ce+=a.position[1];n++}});if(n){cn/=n;ce/=n}let X=e=>W/2+(e-ce)*s,Y=q=>H/2-(q-cn)*s;
 x.strokeStyle='#1c293b';x.lineWidth=1;for(let i=-30;i<=30;i+=5){x.beginPath();x.moveTo(X(i),0);x.lineTo(X(i),H);x.stroke();x.beginPath();x.moveTo(0,Y(i));x.lineTo(W,Y(i));x.stroke()}x.fillStyle='#91a0b8';x.font=13*devicePixelRatio+'px system-ui';x.fillText('N ↑',W-55*devicePixelRatio,28*devicePixelRatio);
 if(!state)return;if(state.route&&state.route.waypoints.length>1){x.save();x.strokeStyle='#38d996aa';x.lineWidth=2*devicePixelRatio;x.setLineDash([3*devicePixelRatio,5*devicePixelRatio]);x.beginPath();state.route.waypoints.forEach((p,i)=>i?x.lineTo(X(p[1]),Y(p[0])):x.moveTo(X(p[1]),Y(p[0])));x.stroke();x.restore()}if(state.geofence){let f=state.geofence,m=f.margin_m||0;x.save();x.lineWidth=2*devicePixelRatio;x.strokeStyle='#ffbd59';x.setLineDash([]);
  const ring=(r,dash)=>{x.setLineDash(dash?[6*devicePixelRatio,6*devicePixelRatio]:[]);x.beginPath();x.arc(X(f.center[1]),Y(f.center[0]),r*s,0,Math.PI*2);x.stroke()};
  const rect=(a,dash)=>{x.setLineDash(dash?[6*devicePixelRatio,6*devicePixelRatio]:[]);x.strokeRect(X(a[1]),Y(a[2]),(a[3]-a[1])*s,(a[2]-a[0])*s)};
  if(f.kind==='circle'&&f.radius_m<1e5){ring(f.radius_m,false);ring(f.radius_m-m,true)}else if(f.kind==='box'){rect(f.box,false);rect([f.box[0]+m,f.box[1]+m,f.box[2]-m,f.box[3]-m],true)}x.restore()}
 if(!state)return;state.obstacles.forEach(o=>{x.beginPath();x.arc(X(o[1]),Y(o[0]),o[2]*s,0,Math.PI*2);x.fillStyle='#ff647426';x.fill();x.strokeStyle='#ff6474';x.stroke()});
 (state.sensed_obstacles||[]).forEach(o=>{x.beginPath();x.setLineDash([4,3]);x.arc(X(o[1]),Y(o[0]),Math.max(o[2]*s,4),0,Math.PI*2);x.strokeStyle='#ff9f43';x.lineWidth=2*devicePixelRatio;x.stroke();x.setLineDash([])});
 let gx=X(state.goal[1]),gy=Y(state.goal[0]);x.strokeStyle='#ffbd59';x.lineWidth=2*devicePixelRatio;x.beginPath();x.arc(gx,gy,8*devicePixelRatio,0,Math.PI*2);x.moveTo(gx-13*devicePixelRatio,gy);x.lineTo(gx+13*devicePixelRatio,gy);x.moveTo(gx,gy-13*devicePixelRatio);x.lineTo(gx,gy+13*devicePixelRatio);x.stroke();
 let byId=Object.fromEntries(state.agents.filter(a=>a.position).map(a=>[a.id,a]));x.setLineDash([5*devicePixelRatio,5*devicePixelRatio]);x.strokeStyle='#62a8ff88';x.lineWidth=1.5*devicePixelRatio;state.agents.forEach(a=>{if(!a.position)return;a.peers.forEach(pid=>{let b=byId[pid];if(!b||a.id>pid)return;x.beginPath();x.moveTo(X(a.position[1]),Y(a.position[0]));x.lineTo(X(b.position[1]),Y(b.position[0]));x.stroke()})});x.setLineDash([]);
 state.agents.forEach(a=>{if(!a.position)return;(trail[a.id]??=[]).push([a.position[0],a.position[1]]);if(trail[a.id].length>120)trail[a.id].shift();x.beginPath();trail[a.id].forEach((p,i)=>i?x.lineTo(X(p[1]),Y(p[0])):x.moveTo(X(p[1]),Y(p[0])));x.strokeStyle='#62a8ff55';x.stroke()});
 state.agents.forEach(a=>{if(!a.position)return;let px=X(a.position[1]),py=Y(a.position[0]);x.beginPath();x.arc(px,py,11*devicePixelRatio,0,Math.PI*2);x.fillStyle=a.health==='OK'?'#38d996':'#ffbd59';x.fill();x.strokeStyle=a.leader?'#fff':'#102030';x.lineWidth=a.leader?3:1;x.stroke();x.fillStyle='#081018';x.font='bold '+11*devicePixelRatio+'px system-ui';x.textAlign='center';x.fillText(a.id,px,py+4*devicePixelRatio)});x.textAlign='left'}poll();draw();
</script></body></html>'''


def _coordinator_summary(latest: dict) -> dict:
    """Who the swarm currently follows, from the agents' own reports."""
    sources, acting, term = [], [], 0
    for index, status in latest.items():
        info = status.get("coordinator") or {}
        if info.get("source"):
            sources.append(str(info["source"]))
        if info.get("acting"):
            acting.append(int(index) + 1)
        if isinstance(info.get("term"), int):
            term = max(term, info["term"])
    common = max(set(sources), key=sources.count) if sources else None
    return {"source": common, "acting": sorted(acting), "term": term}


def _sensed_obstacles(latest: dict) -> list:
    """Obstacles the drones found with their (simulated) range sensors, merged across drones."""
    found = []
    for status in latest.values():
        for item in (status.get("obstacle_sensing") or {}).get("items", []) or []:
            try:
                n, e, r = (float(v) for v in item)
            except (TypeError, ValueError):
                continue
            if all(abs(n - f[0]) > 0.75 or abs(e - f[1]) > 0.75 for f in found):
                found.append([n, e, r])
    return found[:32]


def _realism(session) -> dict:
    from sitl.formation_library import formation_scale
    try:
        scale = formation_scale()
    except ValueError:
        scale = None
    return {"profile": getattr(getattr(session, "args", None), "sitl_sensor_profile", None),
            "position_sigma_m": getattr(session, "position_sigma_m", 0.0), "formation_scale": scale}


_CAPTURE_NAME = re.compile(r"^[0-9]+_[A-Za-z0-9_-]{1,40}_D[0-9]{1,2}\.png$")


def _latest_captures(session) -> dict | None:
    """The most recent camera capture round (from the console's captures/index.jsonl)."""
    run_dir = getattr(session, "run_dir", None)
    if run_dir is None:
        return None
    index = Path(run_dir) / "captures" / "index.jsonl"
    try:
        last = index.read_text(encoding="utf-8").strip().splitlines()[-1]
        entry = json.loads(last)
    except (OSError, IndexError, ValueError):
        return None
    return {"label": entry.get("label"), "utc": entry.get("utc"),
            "frames": [{"drone": f.get("drone"), "file": f.get("file")} for f in entry.get("frames", [])
                       if f.get("file") and _CAPTURE_NAME.match(str(f.get("file")))]}


def dashboard_state(session) -> dict:
    if not hasattr(session, "latest"):
        now = time.time()
        count = int(getattr(session, "count", getattr(session, "n", 0)))
        positions = list(getattr(session, "last_pos", []))
        position_times = list(getattr(session, "last_pos_at", []))
        airborne = list(getattr(session, "is_airborne", []))
        armed_states = list(getattr(session, "last_armed_state", []))
        health_flags = list(getattr(session, "drone_health", []))
        faults = list(getattr(session, "simulated_faults", []))
        agents = []
        active_ids = [i for i in range(count) if i < len(health_flags) and
                      health_flags[i] and i < len(faults) and not faults[i]]
        leader = int(getattr(session, "current_leader_idx", 0)) + 1
        guidance = getattr(session, "decentralized_guidance", None)
        for index in range(count):
            received = position_times[index] if index < len(position_times) else 0.0
            stale = not received or now - received > 3.0
            vehicle_armed = armed_states[index] if index < len(armed_states) else None
            vehicle_airborne = bool(airborne[index]) if index < len(airborne) else False
            online = (index < len(health_flags) and health_flags[index] and
                      index < len(faults) and not faults[index])
            position = positions[index].tolist() if index < len(positions) else None
            if position is not None and len(position) != 3:
                position = None
            peers = []
            if guidance is not None:
                try:
                    peers = [int(peer) for peer in guidance.peer_bus.neighbors(index, now)]
                except (AttributeError, TypeError, ValueError):
                    pass
            agents.append({
                "id": index + 1, "health": "STALE" if stale else ("OK" if online else "CHECK"),
                "airborne": vehicle_airborne, "armed": vehicle_armed is True,
                "status_known": vehicle_armed is False,
                "armed_known": vehicle_armed is not None,
                "position": position, "safety": "nominal" if online else "vehicle_failed",
                "formation": getattr(session, "current_formation_style", "v"),
                "agent_online": bool(online), "agent_action_enabled": True,
                "leader": index + 1 == leader, "members": len(active_ids),
                "peers": [peer + 1 for peer in peers], "live_peers": [peer + 1 for peer in peers],
            })
        centroid = getattr(session, "centroid_pos", [0.0, 0.0, -5.0])
        ready = bool(agents) and all(
            agent["health"] != "STALE" and agent["status_known"] and
            not agent["airborne"] and not agent["armed"] for agent in agents
        )
        busy = list(getattr(session, "busy", [False] * count))
        airborne_ready = bool(agents) and all(
            agent["health"] == "OK" and agent["armed_known"] and agent["airborne"] and
            agent["armed"] and now - position_times[index] <= 1.0 and
            not (busy[index] if index < len(busy) else False)
            for index, agent in enumerate(agents)
        ) and not any(getattr(session, "simulated_faults", [False] * count))
        return {
            "control_mode": session.control_mode, "count": count,
            "healthy": sum(agent["health"] == "OK" for agent in agents),
            "airborne": sum(agent["airborne"] for agent in agents),
            "armed": sum(agent["armed"] for agent in agents),
            "formations": list(STYLES), "formation_labels": STYLE_LABELS,
            "formation_change_enabled": True, "formation_ready": ready,
            "formation_change_allowed": ready or airborne_ready,
            "formation_transitioning": bool(getattr(session, "morph_steps_remaining", 0)),
            "leader": leader, "formation": getattr(session, "current_formation_style", "v"),
            "goal": [float(centroid[0]), float(centroid[1])],
            "altitude": -float(centroid[2]), "coordinator_stream": True,
            "peer_fault_enabled": guidance is not None,
            "recovery_label": "Fail vehicle" if not guidance else "Kill agent",
            "recovery_help": ("Centralized failure injection commands LAND to the selected vehicle."
                               if not guidance else
                               "Kill/restart affects the software agent; the SITL flight controller stays running."),
            "agents": agents, "obstacles": [],
            "minimum_separation_m": _minimum_separation(agents),
        }

    advance = getattr(session, "_advance", None)
    if advance is not None:
        advance()               # replay sessions move their cursor lazily on each state request
    now = time.time()
    with session.lock:
        latest = {key: value.copy() for key, value in session.latest.items()}
    agents = []
    leaders = []
    for index in range(session.count):
        status = latest.get(index, {})
        received = float(status.get("operator_received_at", 0.0) or 0.0)
        stale = not received or now - received > 3.0
        safety = str(status.get("safety_state", "unknown"))
        health = "STALE" if stale else ("OK" if safety == "nominal" else "CHECK")
        leader_id = status.get("elected_leader_id")
        if not stale and isinstance(leader_id, int):
            leaders.append(leader_id + 1)
        position = status.get("position")
        if not (isinstance(position, list) and len(position) == 3):
            position = None
        agents.append({
            "id": index + 1, "health": health, "airborne": bool(status.get("airborne")),
            "armed": bool(status.get("armed")), "position": position, "safety": safety,
            "formation_transition_active": bool(status.get("formation_transition_active", False)),
            "formation_transition_progress": status.get("formation_transition_progress"),
            "position_age_s": status.get("position_age_s"),
            "peer_links_ok": set(status.get("peer_ids", ())) <= set(status.get("peer_live_ids", ())),
            "all_members_present": len(status.get("active_member_ids", [])) == session.count,
            "formation": status.get("formation"),
            "agent_online": (index < len(getattr(session, "nodes", [])) and
                             session.nodes[index].poll() is None),
            "agent_action_enabled": not bool(getattr(session, "external_agents", False)),
            "leader": isinstance(leader_id, int) and leader_id == index,
            "members": len(status.get("active_member_ids", [])),
            "peers": [int(value) + 1 for value in status.get("peer_ids", [])],
            "live_peers": [int(value) + 1 for value in status.get("peer_live_ids", [])],
            "estimate": (status.get("estimate_health") or {}).get("state", "ok"),
            "obstacles_known": int((status.get("obstacle_sensing") or {}).get("known", 0) or 0),
            "filter_active": bool((status.get("safety_filter") or {}).get("active")),
        })
    obstacles = []
    for raw in getattr(session.args, "obstacle", ()):
        try:
            obstacles.append([float(value) for value in str(raw).split(",")])
        except ValueError:
            pass
    common_leader = max(set(leaders), key=leaders.count) if leaders else None
    formation_ready = bool(agents) and all(
        agent["health"] == "OK" and not agent["airborne"] and not agent["armed"]
        for agent in agents
    )
    airborne_ready = bool(agents) and all(
        agent["health"] == "OK" and agent["airborne"] and agent["armed"] and
        agent["safety"] == "nominal" and agent["peer_links_ok"] and
        agent["all_members_present"] and agent["position_age_s"] is not None and
        0.0 <= float(agent["position_age_s"]) <= 1.0
        for agent in agents
    )
    return {
        "control_mode": session.control_mode,
        "count": session.count,
        "healthy": sum(agent["health"] == "OK" for agent in agents),
        "airborne": sum(agent["airborne"] for agent in agents),
        "armed": sum(agent["armed"] for agent in agents),
        "formations": list(STYLES),
        "formation_labels": STYLE_LABELS,
        "formation_change_enabled": bool(getattr(session.args, "adaptive_swarm", False)),
        "formation_ready": formation_ready,
        "formation_change_allowed": formation_ready or airborne_ready,
        "formation_transitioning": any(agent["formation_transition_active"] for agent in agents),
        "formation_transition_progress": (
            min(float(agent["formation_transition_progress"]) for agent in agents
                if isinstance(agent["formation_transition_progress"], (int, float)))
            if any(isinstance(agent["formation_transition_progress"], (int, float))
                   for agent in agents) else None
        ),
        "metrics": (session.metrics.snapshot() if getattr(session, "metrics", None) is not None else None),
        "replay": (session.replay_info() if hasattr(session, "replay_info") else None),
        "coordinator": _coordinator_summary(latest),
        "route": (None if getattr(session, "route", None) is None else
                  {"waypoints": [list(p) for p in session.route.waypoints],
                   "loop_start": session.route.loop_start, "speed_m_s": session.route.speed_m_s}),
        "geofence": (session.geofence.describe() if getattr(session, "geofence", None) is not None else None),
        "mode_switch_enabled": hasattr(session, "set_control_mode"),
        "mode_switching": bool(getattr(getattr(session, "mode_switch", None), "busy", False)),
        "mode_switch_state": getattr(getattr(session, "mode_switch", None), "state", None),
        "mode_switch_target": (getattr(getattr(getattr(session, "mode_switch", None), "plan", None),
                                       "to_mode", None)),
        "peer_fault_enabled": True,
        "recovery_label": "Kill agent",
        "recovery_help": "Kill/restart affects the software agent; the SITL flight controller stays running.",
        "leader": common_leader,
        "formation": session.formation,
        "goal": [float(session.goal_ne[0]), float(session.goal_ne[1])],
        "altitude": float(session.target_alt_m),
        "coordinator_stream": bool(session.intent_enabled),
        "agents": agents, "obstacles": obstacles,
        "sensed_obstacles": _sensed_obstacles(latest),
        "realism": _realism(session),
        "captures": _latest_captures(session),
        "minimum_separation_m": _minimum_separation(agents),
    }


def start_dashboard(session, port: int, *, state_provider=None, mode_switch=None, command_proxy=None):
    action_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def send_value(self, value, content_type="application/json", status=200):
            data = value if isinstance(value, bytes) else value.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/replay" and hasattr(session, "replay_info"):
                self.send_value(json.dumps(session.replay_info(), allow_nan=False))
            elif self.path.startswith("/captures/") and getattr(session, "run_dir", None) is not None:
                name = self.path[len("/captures/"):]
                path = Path(session.run_dir) / "captures" / name
                if not _CAPTURE_NAME.match(name) or not path.is_file():
                    self.send_value('{"message":"not found"}', status=404)
                    return
                self.send_value(path.read_bytes(), "image/png")
            elif self.path == "/state":
                state = state_provider() if state_provider else dashboard_state(session)
                self.send_value(json.dumps(state, allow_nan=False))
            else:
                self.send_value(PAGE, "text/html; charset=utf-8")

        def do_POST(self):
            if self.path == "/replay" and hasattr(session, "replay_post"):
                try:
                    size = min(int(self.headers.get("Content-Length", "0")), 1024)
                    self.send_value(json.dumps(session.replay_post(json.loads(self.rfile.read(size))),
                                               allow_nan=False))
                except (ValueError, KeyError, TypeError) as exc:
                    self.send_value(json.dumps({"message": f"bad replay request: {exc}"}), status=400)
                return
            if self.path == "/mode":
                try:
                    size = min(int(self.headers.get("Content-Length", "0")), 1024)
                    mode = str(json.loads(self.rfile.read(size))["mode"]).strip().lower()
                    if mode not in {"centralized", "decentralized", "hybrid"}:
                        raise ValueError("unknown control architecture")
                    state = state_provider() if state_provider else dashboard_state(session)
                    if state.get("airborne", 0) or state.get("armed", 0):
                        if hasattr(session, "set_control_mode"):
                            result = session.set_control_mode(mode)
                            self.send_value(json.dumps(result), status=202 if result.get("accepted") else 409)
                            return
                        self.send_value(json.dumps({
                            "message": "Land and disarm all drones before changing architecture."
                        }), status=409)
                        return
                    if mode_switch is not None:
                        result = mode_switch(mode)
                        self.send_value(json.dumps(result), status=int(result.get("status", 202)))
                        return
                    command = session.mode_relaunch_command(mode)
                    self.send_value(json.dumps({
                        "message": (f"{mode.title()} selected. Stop this grounded console, "
                                    "then run the copied relaunch command."),
                        "command": command,
                    }))
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                    self.send_value(json.dumps({"message": str(exc)}), status=400)
                return
            if self.path == "/formation":
                try:
                    size = min(int(self.headers.get("Content-Length", "0")), 1024)
                    formation = str(json.loads(self.rfile.read(size))["formation"]).strip().lower()
                    if formation not in STYLES:
                        raise ValueError("unknown formation shape")
                    state = state_provider() if state_provider else dashboard_state(session)
                    if not state["formation_change_enabled"]:
                        self.send_value(json.dumps({"message": "Formation changes require adaptive swarm mode."}), status=409)
                        return
                    if not state.get("formation_change_allowed") or state.get("formation_transitioning"):
                        self.send_value(json.dumps({"message": "Need fresh safe status from every drone; wait for the current morph to finish."}), status=409)
                        return
                    if not action_lock.acquire(blocking=False):
                        self.send_value(json.dumps({"message": "Another command is still running."}), status=409)
                        return
                    try:
                        if command_proxy is not None:
                            result = command_proxy("shape " + formation)
                        elif hasattr(session, "_set_formation"):
                            session._set_formation(formation)
                            result = getattr(session, "last_formation_result", {
                                "accepted": False,
                                "message": "Formation request did not return a result.",
                            })
                        else:
                            session.set_formation_style(formation)
                            result = getattr(session, "last_formation_result", {
                                "accepted": False,
                                "message": "Formation request did not return a result.",
                            })
                        accepted = bool(result.get("accepted", False))
                        status = int(result.get("status", 200 if accepted else 409))
                        self.send_value(json.dumps(result), status=status)
                    finally:
                        action_lock.release()
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                    self.send_value(json.dumps({"message": str(exc)}), status=400)
                return
            if self.path != "/command":
                self.send_value('{"message":"not found"}', status=404)
                return
            try:
                size = min(int(self.headers.get("Content-Length", "0")), 1024)
                command = str(json.loads(self.rfile.read(size))["command"]).strip()
                allowed = {"launch", "land", "status", "preflight", "w 5", "s 5", "a 5", "d 5",
                           "u 2", "j 2", "patrol square 12", "route stop", "goto 0 0", "capture dashboard",
                           "coord off", "coord on"}
                parts = command.split()
                valid_link = (len(parts) == 3 and parts[0] in {"link-block", "link-restore"}
                              and all(value.isdigit() for value in parts[1:]))
                valid_agent_action = (len(parts) == 2 and parts[0] in {"fail", "restart"}
                                      and parts[1].isdigit()
                                      and 1 <= int(parts[1]) <= session.count)
                if command not in allowed and not valid_link and not valid_agent_action:
                    raise ValueError("command is not available on the dashboard")
                if not action_lock.acquire(blocking=False):
                    self.send_value('{"message":"Another command is still running"}', status=409)
                    return
                try:
                    if command_proxy is not None:
                        result = command_proxy(command)
                    else:
                        session.last_command_result = None
                        session.handle(command)
                        result = session.last_command_result or {
                            "accepted": True, "message": f"Command accepted: {command}",
                        }
                    accepted = bool(result.get("accepted", True))
                    status = int(result.get("status", 200 if accepted else 409))
                    result.setdefault("message", f"Command accepted: {command}")
                    self.send_value(json.dumps(result), status=status)
                finally:
                    action_lock.release()
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self.send_value(json.dumps({"message": str(exc)}), status=400)

    server = ThreadingHTTPServer(("127.0.0.1", int(port)), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
