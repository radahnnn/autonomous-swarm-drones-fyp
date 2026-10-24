"""Small, readable browser dashboard for the independent swarm operator."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time


PAGE = r'''<!doctype html><html><head><meta charset="utf-8"><title>Swarm Control</title>
<style>
:root{--bg:#0c111b;--card:#151d2b;--edge:#28364b;--text:#eef4ff;--muted:#91a0b8;--ok:#38d996;--warn:#ffbd59;--bad:#ff6474;--blue:#62a8ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,sans-serif;height:100vh;display:grid;grid-template-columns:minmax(0,1fr) 360px}
main{position:relative;min-width:0}canvas{width:100%;height:100%;display:block}.top{position:absolute;top:18px;left:18px;right:18px;display:flex;gap:10px;pointer-events:none}.pill{background:#111a28e8;border:1px solid var(--edge);border-radius:12px;padding:10px 14px;box-shadow:0 8px 30px #0005}.pill b{font-size:18px}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
aside{background:var(--card);border-left:1px solid var(--edge);padding:20px;overflow:auto}h1{font-size:20px;margin:0 0 4px}h2{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.11em;margin:22px 0 9px}.sub{color:var(--muted);font-size:13px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}.grid.two{grid-template-columns:1fr 1fr}button{border:1px solid var(--edge);background:#202c3e;color:var(--text);padding:11px 8px;border-radius:9px;font-weight:650;cursor:pointer}button:hover{background:#2a3a51}button:disabled{opacity:.42;cursor:not-allowed}.modebtn.active{border-color:var(--blue);background:#17375d;box-shadow:0 0 0 1px var(--blue) inset}.launch{background:#176c4b}.land{background:#762c38}.agentsgrid{display:grid;grid-template-columns:1fr 1fr;gap:7px}.card{background:#101824;border:1px solid var(--edge);border-radius:10px;padding:9px;margin:0;min-width:0}.row{display:flex;justify-content:space-between;gap:6px}.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:6px;background:var(--ok)}input{width:100%;background:#0d1420;color:var(--text);border:1px solid var(--edge);border-radius:8px;padding:9px}.fault{display:grid;grid-template-columns:1fr 1fr;gap:7px}.msg{min-height:36px;color:var(--muted);font-size:13px;margin-top:10px}details{margin-top:18px;color:var(--muted)}pre{white-space:pre-wrap;font-size:11px}
</style></head><body><main><canvas id="map"></canvas><div class="top"><div class="pill"><b id="mode">CONNECTING</b><br><span id="mission" class="sub">Waiting for status</span></div><div class="pill"><b id="health">0 / 0</b><br><span class="sub">healthy agents</span></div><div class="pill"><b id="leader">—</b><br><span class="sub">elected leader</span></div></div></main>
<aside><h1>Swarm Control</h1><div class="sub">Simple operator view · map shows local North/East</div>
<h2>Control architecture</h2><div class="grid"><button class="modebtn" data-mode="centralized" onclick="chooseMode('centralized')">Centralized</button><button class="modebtn" data-mode="decentralized" onclick="chooseMode('decentralized')">Decentralized</button><button class="modebtn" data-mode="hybrid" onclick="chooseMode('hybrid')">Hybrid</button></div><div id="modehelp" class="sub" style="margin-top:7px">Loading active architecture…</div>
<h2>Flight</h2><div class="grid two"><button class="launch" onclick="send('launch')">Launch all</button><button class="land" onclick="land()">Land all</button></div>
<h2>Move swarm</h2><div class="grid"><span></span><button onclick="send('w 5')">↑ North</button><span></span><button onclick="send('a 5')">← West</button><button onclick="send('status')">Refresh</button><button onclick="send('d 5')">East →</button><span></span><button onclick="send('s 5')">↓ South</button><span></span></div>
<h2>Altitude</h2><div class="grid two"><button onclick="send('u 2')">Up 2 m</button><button onclick="send('j 2')">Down 2 m</button></div>
<h2>Communication test</h2><div class="fault"><input id="from" type="number" min="1" value="1" aria-label="sender"><input id="to" type="number" min="1" value="2" aria-label="recipient"><button onclick="link(true)">Block link</button><button onclick="link(false)">Restore link</button></div><div class="sub" style="margin-top:7px">Cuts one direction only. The remaining drones should keep flying.</div>
<h2>Drone health</h2><div id="agents" class="agentsgrid"></div><div class="msg" id="msg"></div>
<details><summary>Technical details</summary><pre id="raw"></pre></details></aside>
<script>
let state=null,trail={};const c=document.getElementById('map'),x=c.getContext('2d');
function resize(){c.width=c.clientWidth*devicePixelRatio;c.height=c.clientHeight*devicePixelRatio}addEventListener('resize',resize);resize();
async function send(command){msg.textContent='Sending: '+command;try{let r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command})});let j=await r.json();msg.textContent=j.message}catch(e){msg.textContent='Command failed: '+e}}
function land(){if(confirm('Land all drones now?'))send('land')}function link(block){let a=from.value,b=to.value;send((block?'link-block ':'link-restore ')+a+' '+b)}
async function chooseMode(choice){if(!state)return;if(state.airborne){msg.textContent='Land all drones before changing architecture.';return}if(choice===state.control_mode){msg.textContent=choice+' is already active.';return}try{let r=await fetch('/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:choice})});let j=await r.json();msg.textContent=j.message;if(j.command&&navigator.clipboard){await navigator.clipboard.writeText(j.command);msg.textContent+=' Relaunch command copied.'}}catch(e){msg.textContent='Mode selection failed: '+e}}
async function poll(){try{let r=await fetch('/state',{cache:'no-store'});state=await r.json();update()}catch(e){mode.textContent='DISCONNECTED';mode.className='bad'}setTimeout(poll,350)}
function update(){mode.textContent=state.control_mode.toUpperCase();mode.className=state.healthy===state.count?'ok':'warn';health.textContent=state.healthy+' / '+state.count;leader.textContent=state.leader==null?'—':'D'+state.leader;mission.textContent=state.airborne+' airborne · '+state.formation.toUpperCase()+' · target N '+state.goal[0].toFixed(1)+' E '+state.goal[1].toFixed(1)+' · '+state.altitude.toFixed(1)+' m';
 document.querySelectorAll('.modebtn').forEach(b=>{let active=b.dataset.mode===state.control_mode;b.classList.toggle('active',active);b.disabled=state.airborne>0&&!active});modehelp.textContent=state.airborne?'Architecture is locked while airborne. Land before switching.':'Select another mode to prepare its safe relaunch command.';
 agents.innerHTML=state.agents.map(a=>'<div class="card"><div class="row"><b><span class="dot" style="background:'+(a.health==='OK'?'var(--ok)':a.health==='STALE'?'var(--bad)':'var(--warn)')+'"></span>D'+a.id+(a.leader?' ★':'')+'</b><b>'+a.health+'</b></div><div class="row sub"><span>'+(a.airborne?'Airborne':'Grounded')+' · '+a.safety+'</span><span>'+a.members+' members</span></div></div>').join('');raw.textContent=JSON.stringify(state,null,2)}
function draw(){requestAnimationFrame(draw);x.clearRect(0,0,c.width,c.height);let s=18*devicePixelRatio,W=c.width,H=c.height,cn=0,ce=0,n=0;if(state)state.agents.forEach(a=>{if(a.position){cn+=a.position[0];ce+=a.position[1];n++}});if(n){cn/=n;ce/=n}let X=e=>W/2+(e-ce)*s,Y=q=>H/2-(q-cn)*s;
 x.strokeStyle='#1c293b';x.lineWidth=1;for(let i=-30;i<=30;i+=5){x.beginPath();x.moveTo(X(i),0);x.lineTo(X(i),H);x.stroke();x.beginPath();x.moveTo(0,Y(i));x.lineTo(W,Y(i));x.stroke()}x.fillStyle='#91a0b8';x.font=13*devicePixelRatio+'px system-ui';x.fillText('N ↑',W-55*devicePixelRatio,28*devicePixelRatio);
 if(!state)return;state.obstacles.forEach(o=>{x.beginPath();x.arc(X(o[1]),Y(o[0]),o[2]*s,0,Math.PI*2);x.fillStyle='#ff647426';x.fill();x.strokeStyle='#ff6474';x.stroke()});
 let gx=X(state.goal[1]),gy=Y(state.goal[0]);x.strokeStyle='#ffbd59';x.lineWidth=2*devicePixelRatio;x.beginPath();x.arc(gx,gy,8*devicePixelRatio,0,Math.PI*2);x.moveTo(gx-13*devicePixelRatio,gy);x.lineTo(gx+13*devicePixelRatio,gy);x.moveTo(gx,gy-13*devicePixelRatio);x.lineTo(gx,gy+13*devicePixelRatio);x.stroke();
 let byId=Object.fromEntries(state.agents.filter(a=>a.position).map(a=>[a.id,a]));x.setLineDash([5*devicePixelRatio,5*devicePixelRatio]);x.strokeStyle='#62a8ff88';x.lineWidth=1.5*devicePixelRatio;state.agents.forEach(a=>{if(!a.position)return;a.peers.forEach(pid=>{let b=byId[pid];if(!b||a.id>pid)return;x.beginPath();x.moveTo(X(a.position[1]),Y(a.position[0]));x.lineTo(X(b.position[1]),Y(b.position[0]));x.stroke()})});x.setLineDash([]);
 state.agents.forEach(a=>{if(!a.position)return;(trail[a.id]??=[]).push([a.position[0],a.position[1]]);if(trail[a.id].length>120)trail[a.id].shift();x.beginPath();trail[a.id].forEach((p,i)=>i?x.lineTo(X(p[1]),Y(p[0])):x.moveTo(X(p[1]),Y(p[0])));x.strokeStyle='#62a8ff55';x.stroke()});
 state.agents.forEach(a=>{if(!a.position)return;let px=X(a.position[1]),py=Y(a.position[0]);x.beginPath();x.arc(px,py,11*devicePixelRatio,0,Math.PI*2);x.fillStyle=a.health==='OK'?'#38d996':'#ffbd59';x.fill();x.strokeStyle=a.leader?'#fff':'#102030';x.lineWidth=a.leader?3:1;x.stroke();x.fillStyle='#081018';x.font='bold '+11*devicePixelRatio+'px system-ui';x.textAlign='center';x.fillText(a.id,px,py+4*devicePixelRatio)});x.textAlign='left'}poll();draw();
</script></body></html>'''


def dashboard_state(session) -> dict:
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
            "leader": isinstance(leader_id, int) and leader_id == index,
            "members": len(status.get("active_member_ids", [])),
            "peers": [int(value) + 1 for value in status.get("peer_ids", [])],
            "live_peers": [int(value) + 1 for value in status.get("peer_live_ids", [])],
        })
    obstacles = []
    for raw in getattr(session.args, "obstacle", ()):
        try:
            obstacles.append([float(value) for value in str(raw).split(",")])
        except ValueError:
            pass
    common_leader = max(set(leaders), key=leaders.count) if leaders else None
    return {
        "control_mode": session.control_mode,
        "count": session.count,
        "healthy": sum(agent["health"] == "OK" for agent in agents),
        "airborne": sum(agent["airborne"] for agent in agents),
        "leader": common_leader,
        "formation": session.formation,
        "goal": [float(session.goal_ne[0]), float(session.goal_ne[1])],
        "altitude": float(session.target_alt_m),
        "coordinator_stream": bool(session.intent_enabled),
        "agents": agents, "obstacles": obstacles,
    }


def start_dashboard(session, port: int):
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
            if self.path == "/state":
                self.send_value(json.dumps(dashboard_state(session), allow_nan=False))
            else:
                self.send_value(PAGE, "text/html; charset=utf-8")

        def do_POST(self):
            if self.path == "/mode":
                try:
                    size = min(int(self.headers.get("Content-Length", "0")), 1024)
                    mode = str(json.loads(self.rfile.read(size))["mode"]).strip().lower()
                    if mode not in {"centralized", "decentralized", "hybrid"}:
                        raise ValueError("unknown control architecture")
                    state = dashboard_state(session)
                    if state["airborne"]:
                        self.send_value(json.dumps({
                            "message": "Land all drones before changing architecture."
                        }), status=409)
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
            if self.path != "/command":
                self.send_value('{"message":"not found"}', status=404)
                return
            try:
                size = min(int(self.headers.get("Content-Length", "0")), 1024)
                command = str(json.loads(self.rfile.read(size))["command"]).strip()
                allowed = {"launch", "land", "status", "w 5", "s 5", "a 5", "d 5",
                           "u 2", "j 2"}
                parts = command.split()
                valid_link = (len(parts) == 3 and parts[0] in {"link-block", "link-restore"}
                              and all(value.isdigit() for value in parts[1:]))
                if command not in allowed and not valid_link:
                    raise ValueError("command is not available on the dashboard")
                if not action_lock.acquire(blocking=False):
                    self.send_value('{"message":"Another command is still running"}', status=409)
                    return
                def run():
                    try:
                        session.handle(command)
                    finally:
                        action_lock.release()
                threading.Thread(target=run, daemon=True).start()
                self.send_value(json.dumps({"message": f"Sent: {command}"}))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self.send_value(json.dumps({"message": str(exc)}), status=400)

    server = ThreadingHTTPServer(("127.0.0.1", int(port)), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
