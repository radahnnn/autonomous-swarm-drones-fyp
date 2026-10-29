"""Small, readable browser dashboard for the independent swarm operator."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

from sitl.formation_library import STYLES


STYLE_LABELS = {
    "v": "V", "line": "Line", "column": "Column", "diamond": "Diamond",
    "ring": "Ring", "echelon": "Echelon", "triangle": "Triangle",
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
</style></head><body><main><canvas id="map"></canvas><div class="top"><div class="pill"><b id="mode">CONNECTING</b><br><span id="mission" class="sub">Waiting for status</span></div><div class="pill"><b id="health">0 / 0</b><br><span class="sub">healthy agents</span></div><div class="pill"><b id="leader">—</b><br><span class="sub">elected leader</span></div></div></main>
<aside><h1>Swarm Control</h1><div class="sub">Simple operator view · map shows local North/East</div>
<h2>Control architecture</h2><div class="grid"><button class="modebtn" data-mode="centralized" onclick="chooseMode('centralized')">Centralized</button><button class="modebtn" data-mode="decentralized" onclick="chooseMode('decentralized')">Decentralized</button><button class="modebtn" data-mode="hybrid" onclick="chooseMode('hybrid')">Hybrid</button></div><div id="modehelp" class="sub" style="margin-top:7px">Loading active architecture…</div>
<h2>Flight</h2><div class="grid"><button onclick="send('preflight')">Check safety</button><button class="launch" onclick="send('launch')">Launch all</button><button class="land" onclick="land()">Land all</button></div>
<h2>Move swarm</h2><div class="grid"><span></span><button onclick="send('w 5')">↑ North</button><span></span><button onclick="send('a 5')">← West</button><button onclick="send('status')">Refresh</button><button onclick="send('d 5')">East →</button><span></span><button onclick="send('s 5')">↓ South</button><span></span></div>
<h2>Formation shape</h2><div class="grid" id="shapes"></div><div class="sub" id="shapehelp">Shape changes are available grounded, or in flight after safety checks.</div>
<h2>Altitude</h2><div class="grid two"><button onclick="send('u 2')">Up 2 m</button><button onclick="send('j 2')">Down 2 m</button></div>
<h2>Communication test</h2><div class="fault"><input id="from" type="number" min="1" value="1" aria-label="sender"><input id="to" type="number" min="1" value="2" aria-label="recipient"><button id="blocklink" onclick="link(true)">Block link</button><button id="restorelink" onclick="link(false)">Restore link</button></div><div class="sub" style="margin-top:7px">Cuts one direction only. The remaining drones should keep flying.</div>
<h2>Drone health & recovery</h2><div class="sub" id="recoveryhelp">Recovery controls depend on the active architecture.</div><div id="agents" class="agentsgrid"></div><div class="msg" id="msg"></div>
<details><summary>Technical details</summary><pre id="raw"></pre></details></aside>
<script>
let state=null,trail={};const c=document.getElementById('map'),x=c.getContext('2d');
function resize(){c.width=c.clientWidth*devicePixelRatio;c.height=c.clientHeight*devicePixelRatio}addEventListener('resize',resize);resize();
async function send(command){msg.textContent='Sending: '+command;try{let r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command})});let j=await r.json();let p=j.preflight;if(p){let gap=Number.isFinite(p.minimum_separation_m)?' Minimum separation: '+p.minimum_separation_m.toFixed(2)+' m.':'';let reasons=[...new Set((p.agents||[]).flatMap(a=>a.reasons||[]))];msg.textContent=j.message+gap+(reasons.length?' Reasons: '+reasons.join(', ')+'.':'')}else msg.textContent=j.message}catch(e){msg.textContent='Command failed: '+e}}
function land(){if(confirm('Land all drones now?'))send('land')}function link(block){let a=from.value,b=to.value;send((block?'link-block ':'link-restore ')+a+' '+b)}
async function setShape(style){if(!state||!state.formation_change_allowed||state.formation_transitioning){msg.textContent='Wait for fresh safe status and for any current formation transition to finish.';return}msg.textContent='Planning safe transition to '+style+'…';try{let r=await fetch('/formation',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({formation:style})});let j=await r.json();msg.textContent=j.message}catch(e){msg.textContent='Formation change failed: '+e}}
async function agentAction(id,online){let centralized=state&&state.control_mode==='centralized';let detail=online?(centralized?'Fail vehicle D'+id+'? The centralized console will command LAND for that SITL vehicle.':'Stop D'+id+' software agent? Its SITL flight controller stays active and may hold its last target if airborne.'):(centralized?'Revive vehicle D'+id+' in the swarm?':'Restart D'+id+' software agent? If airborne, the replacement agent will command LAND.');if(!confirm(detail))return;send((online?'fail ':'restart ')+id)}
async function chooseMode(choice){if(!state)return;if(state.airborne||state.armed){msg.textContent='Land and disarm all drones before changing architecture.';return}if(choice===state.control_mode){msg.textContent=choice+' is already active.';return}if(!confirm('Switch to '+choice+' mode? The controller will restart after confirming all drones are grounded and disarmed; SITL and QGroundControl will stay running.'))return;try{let r=await fetch('/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:choice})});let j=await r.json();msg.textContent=j.message}catch(e){msg.textContent='Mode switch failed: '+e}}
async function poll(){try{let r=await fetch('/state',{cache:'no-store'});state=await r.json();update()}catch(e){mode.textContent='DISCONNECTED';mode.className='bad'}setTimeout(poll,350)}
function update(){mode.textContent=state.transitioning?'SWITCHING…':state.control_mode.toUpperCase();mode.className=state.transitioning?'warn':(state.healthy===state.count?'ok':'warn');health.textContent=state.healthy+' / '+state.count;leader.textContent=state.leader==null?'—':'D'+state.leader;let gap=Number.isFinite(state.minimum_separation_m)?' · min gap '+state.minimum_separation_m.toFixed(1)+' m':'';mission.textContent=state.airborne+' airborne · '+state.formation.toUpperCase()+gap+' · target N '+state.goal[0].toFixed(1)+' E '+state.goal[1].toFixed(1)+' · '+state.altitude.toFixed(1)+' m';
 document.querySelectorAll('.modebtn').forEach(b=>{let active=b.dataset.mode===state.control_mode;b.classList.toggle('active',active);b.disabled=!!state.transitioning||state.airborne>0||state.armed>0||!state.healthy});modehelp.textContent=state.transitioning?'Switching safely to '+state.switch_target+'; the dashboard stays connected.':(state.transition_error||'Select another mode to restart the grounded controller while SITL remains running.');
 shapes.innerHTML=state.formations.map(style=>'<button class="shape" onclick="setShape(\''+style+'\')">'+state.formation_labels[style]+'</button>').join('');document.querySelectorAll('.shape').forEach(b=>{b.classList.toggle('active',b.textContent.toLowerCase()===state.formation);b.disabled=!state.formation_change_enabled||!state.formation_change_allowed||!!state.formation_transitioning||!!state.transitioning});shapehelp.textContent=!state.formation_change_enabled?'Formation controls are unavailable in this mode.':(state.formation_transitioning?'Smooth formation transition: '+Math.round(100*(state.formation_transition_progress||0))+'% complete; wait before issuing another change.':(!state.formation_change_allowed?'Waiting for fresh, healthy status and a safe all-grounded or all-airborne swarm.':'Every requested shape is screened for path clearance; unsafe or crowded transitions are rejected.'));
 document.getElementById('blocklink').disabled=!state.peer_fault_enabled||!!state.transitioning;document.getElementById('restorelink').disabled=!state.peer_fault_enabled||!!state.transitioning;recoveryhelp.textContent=state.recovery_help||'Recovery controls depend on the active architecture.';
 agents.innerHTML=state.agents.map(a=>'<div class="card"><div class="row"><b><span class="dot" style="background:'+(a.health==='OK'?'var(--ok)':a.health==='STALE'?'var(--bad)':'var(--warn)')+'"></span>D'+a.id+(a.leader?' ★':'')+'</b><b>'+a.health+'</b></div><div class="row sub"><span>'+(a.airborne?'Airborne':'Grounded')+' · '+a.safety+'</span><span>'+a.members+' members</span></div><div class="row sub"><span>Agent '+(a.agent_online?'online':'stopped')+'</span><span>'+String(a.formation||state.formation).toUpperCase()+'</span></div><button onclick="agentAction('+a.id+','+(a.agent_online?'true':'false')+')" '+(a.agent_action_enabled?'':'disabled')+'>'+(a.agent_online?(state.recovery_label||'Kill agent'):'Revive / restart')+'</button></div>').join('');raw.textContent=JSON.stringify(state,null,2)}
function draw(){requestAnimationFrame(draw);x.clearRect(0,0,c.width,c.height);let s=18*devicePixelRatio,W=c.width,H=c.height,cn=0,ce=0,n=0;if(state)state.agents.forEach(a=>{if(a.position){cn+=a.position[0];ce+=a.position[1];n++}});if(n){cn/=n;ce/=n}let X=e=>W/2+(e-ce)*s,Y=q=>H/2-(q-cn)*s;
 x.strokeStyle='#1c293b';x.lineWidth=1;for(let i=-30;i<=30;i+=5){x.beginPath();x.moveTo(X(i),0);x.lineTo(X(i),H);x.stroke();x.beginPath();x.moveTo(0,Y(i));x.lineTo(W,Y(i));x.stroke()}x.fillStyle='#91a0b8';x.font=13*devicePixelRatio+'px system-ui';x.fillText('N ↑',W-55*devicePixelRatio,28*devicePixelRatio);
 if(!state)return;state.obstacles.forEach(o=>{x.beginPath();x.arc(X(o[1]),Y(o[0]),o[2]*s,0,Math.PI*2);x.fillStyle='#ff647426';x.fill();x.strokeStyle='#ff6474';x.stroke()});
 let gx=X(state.goal[1]),gy=Y(state.goal[0]);x.strokeStyle='#ffbd59';x.lineWidth=2*devicePixelRatio;x.beginPath();x.arc(gx,gy,8*devicePixelRatio,0,Math.PI*2);x.moveTo(gx-13*devicePixelRatio,gy);x.lineTo(gx+13*devicePixelRatio,gy);x.moveTo(gx,gy-13*devicePixelRatio);x.lineTo(gx,gy+13*devicePixelRatio);x.stroke();
 let byId=Object.fromEntries(state.agents.filter(a=>a.position).map(a=>[a.id,a]));x.setLineDash([5*devicePixelRatio,5*devicePixelRatio]);x.strokeStyle='#62a8ff88';x.lineWidth=1.5*devicePixelRatio;state.agents.forEach(a=>{if(!a.position)return;a.peers.forEach(pid=>{let b=byId[pid];if(!b||a.id>pid)return;x.beginPath();x.moveTo(X(a.position[1]),Y(a.position[0]));x.lineTo(X(b.position[1]),Y(b.position[0]));x.stroke()})});x.setLineDash([]);
 state.agents.forEach(a=>{if(!a.position)return;(trail[a.id]??=[]).push([a.position[0],a.position[1]]);if(trail[a.id].length>120)trail[a.id].shift();x.beginPath();trail[a.id].forEach((p,i)=>i?x.lineTo(X(p[1]),Y(p[0])):x.moveTo(X(p[1]),Y(p[0])));x.strokeStyle='#62a8ff55';x.stroke()});
 state.agents.forEach(a=>{if(!a.position)return;let px=X(a.position[1]),py=Y(a.position[0]);x.beginPath();x.arc(px,py,11*devicePixelRatio,0,Math.PI*2);x.fillStyle=a.health==='OK'?'#38d996':'#ffbd59';x.fill();x.strokeStyle=a.leader?'#fff':'#102030';x.lineWidth=a.leader?3:1;x.stroke();x.fillStyle='#081018';x.font='bold '+11*devicePixelRatio+'px system-ui';x.textAlign='center';x.fillText(a.id,px,py+4*devicePixelRatio)});x.textAlign='left'}poll();draw();
</script></body></html>'''


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
        "peer_fault_enabled": True,
        "recovery_label": "Kill agent",
        "recovery_help": "Kill/restart affects the software agent; the SITL flight controller stays running.",
        "leader": common_leader,
        "formation": session.formation,
        "goal": [float(session.goal_ne[0]), float(session.goal_ne[1])],
        "altitude": float(session.target_alt_m),
        "coordinator_stream": bool(session.intent_enabled),
        "agents": agents, "obstacles": obstacles,
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
            if self.path == "/state":
                state = state_provider() if state_provider else dashboard_state(session)
                self.send_value(json.dumps(state, allow_nan=False))
            else:
                self.send_value(PAGE, "text/html; charset=utf-8")

        def do_POST(self):
            if self.path == "/mode":
                try:
                    size = min(int(self.headers.get("Content-Length", "0")), 1024)
                    mode = str(json.loads(self.rfile.read(size))["mode"]).strip().lower()
                    if mode not in {"centralized", "decentralized", "hybrid"}:
                        raise ValueError("unknown control architecture")
                    state = state_provider() if state_provider else dashboard_state(session)
                    if state.get("airborne", 0) or state.get("armed", 0):
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
                           "u 2", "j 2"}
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
