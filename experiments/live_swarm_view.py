#!/usr/bin/env python3
"""
Live top-down view of the swarm (standalone SITL, no GPU / Gazebo needed) with model predictions drawn next to each drone.

    python experiments/live_swarm_view.py [--drones 5] [--port 8765]
    ssh -L 8765:localhost:8765 user@host      # then open http://localhost:8765

The server binds to 127.0.0.1 only. The page shows, per drone, the real SITL position (green), the commanded setpoint (hollow),
the identified autopilot model S4 (red) and the old "fitted" model (blue), both driven live by the same setpoints,
plus running errors. Buttons send the same commands as the swarm console.
"""

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sitl.swarm_console import SwarmConsole, handle_command  # noqa: E402
from swarm_core.autopilot_tracking import AutopilotTrackingModel  # noqa: E402
from swarm_core.drone import Drone  # noqa: E402

PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Swarm live view</title>
<style>
:root{--bg:#0f1218;--panel:#171b24;--line:#2a3140;--tx:#d7dde8;--mut:#8590a3;--ok:#3ddc84;--s4:#ff5a5f;--old:#4da3ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px system-ui,Segoe UI,sans-serif;display:flex;height:100vh}
#main{flex:1;position:relative}canvas{width:100%;height:100%;display:block}
#side{width:340px;background:var(--panel);border-left:1px solid var(--line);padding:12px;overflow:auto}
h3{margin:12px 0 6px;font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--mut)}
button{background:#232a38;color:var(--tx);border:1px solid var(--line);border-radius:6px;padding:7px 10px;margin:2px;cursor:pointer;font-size:13px}
button:hover{background:#2d3648}button.go{background:#1d6b3c;border-color:#2a8f52}button.bad{background:#6b1d1d;border-color:#8f2a2a}
table{width:100%;border-collapse:collapse;font-size:12px}td,th{padding:3px 4px;text-align:right;border-bottom:1px solid var(--line)}th:first-child,td:first-child{text-align:left}
.k{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:5px;vertical-align:middle}
#phase{padding:6px 8px;border-radius:6px;background:#232a38;margin-bottom:6px}#big{font-size:22px;font-weight:600}
</style></head><body>
<div id="main"><canvas id="c"></canvas></div>
<div id="side">
 <div id="phase">connecting...</div>
 <h3>Mission</h3>
 <button class="go" onclick="cmd('launch')">Launch swarm</button><button class="bad" onclick="cmd('q')">Land</button>
 <h3>Move</h3>
 <button onclick="cmd('w 6')">North 6</button><button onclick="cmd('s 6')">South 6</button><button onclick="cmd('a 5')">West 5</button><button onclick="cmd('d 5')">East 5</button><button onclick="cmd('c')">Centre</button>
 <h3>Formation</h3>
 <button onclick="cmd('v')">V</button><button onclick="cmd('l')">Line</button><button onclick="cmd('k')">Column</button><button onclick="cmd('x')">Diamond</button><button onclick="cmd('o')">Ring</button><button onclick="cmd('e')">Echelon</button><button onclick="cmd('tri')">Triangle</button>
 <h3>Fault tolerance</h3>
 <button onclick="cmd('f 1')">Fail D1</button><button onclick="cmd('f 3')">Fail D3</button><button onclick="cmd('r 1')">Revive D1</button><button onclick="cmd('r 3')">Revive D3</button>
 <h3>Model error now (cm, mean over drones)</h3>
 <div><span class="k" style="background:var(--s4)"></span><b id="e4">-</b> new autopilot model &nbsp; <span style="color:var(--mut)">avg <span id="a4">-</span></span></div>
 <div><span class="k" style="background:var(--old)"></span><b id="eo">-</b> old model (tau .992) &nbsp; <span style="color:var(--mut)">avg <span id="ao">-</span></span></div>
 <h3>Drones</h3><table id="tb"></table>
 <h3>Legend</h3>
 <div><span class="k" style="background:var(--ok)"></span>real SITL drone &nbsp;<span class="k" style="border:2px solid #aab;background:none"></span>setpoint</div>
 <div><span class="k" style="background:var(--s4)"></span>new model prediction &nbsp;<span class="k" style="background:var(--old)"></span>old model</div>
 <div id="msg" style="margin-top:8px;color:var(--mut)"></div>
</div>
<script>
const cv=document.getElementById('c'),cx=cv.getContext('2d');let st=null,cam={n:0,e:0},trails=[];
function rs(){cv.width=cv.clientWidth*devicePixelRatio;cv.height=cv.clientHeight*devicePixelRatio}addEventListener('resize',rs);rs();
async function cmd(c){try{const r=await fetch('/cmd',{method:'POST',body:JSON.stringify({c})});document.getElementById('msg').textContent='> '+c+': '+(await r.text())}catch(e){document.getElementById('msg').textContent='error '+e}}
async function poll(){try{const r=await fetch('/state');st=await r.json();upd()}catch(e){document.getElementById('phase').textContent='disconnected'}setTimeout(poll,200)}
function upd(){document.getElementById('phase').innerHTML='<b>'+st.phase+'</b> &middot; formation <b>'+st.style.toUpperCase()+'</b> &middot; airborne '+st.airborne+'/'+st.n;
 document.getElementById('e4').textContent=st.err4.toFixed(1);document.getElementById('eo').textContent=st.erro.toFixed(1);
 document.getElementById('a4').textContent=st.avg4.toFixed(1);document.getElementById('ao').textContent=st.avgo.toFixed(1);
 let h='<tr><th>drone</th><th>alt m</th><th>new</th><th>old</th></tr>';st.drones.forEach(d=>{h+='<tr><td>D'+d.id+(d.leader?' &#9733;':'')+(d.fail?' (lost)':'')+'</td><td>'+d.alt.toFixed(1)+'</td><td>'+(d.e4==null?'-':d.e4.toFixed(0))+'</td><td>'+(d.eo==null?'-':d.eo.toFixed(0))+'</td></tr>'});document.getElementById('tb').innerHTML=h}
function draw(){const W=cv.width,H=cv.height,s=14*devicePixelRatio;cx.clearRect(0,0,W,H);
 if(st){let n=0,e=0,k=0;st.drones.forEach(d=>{if(d.air){n+=d.n;e+=d.e;k++}});if(!k){n=st.drones[0].n;e=st.drones[0].e;k=1}n/=k;e/=k;cam.n+=(n-cam.n)*.1;cam.e+=(e-cam.e)*.1}
 const X=e=>W/2+(e-cam.e)*s,Y=n=>H/2-(n-cam.n)*s;
 cx.strokeStyle='#222a39';cx.lineWidth=1;cx.fillStyle='#566';cx.font=(11*devicePixelRatio)+'px sans-serif';
 const g=5,e0=Math.floor((cam.e-W/2/s)/g)*g,e1=cam.e+W/2/s,n0=Math.floor((cam.n-H/2/s)/g)*g,n1=cam.n+H/2/s;
 for(let a=e0;a<e1;a+=g){cx.beginPath();cx.moveTo(X(a),0);cx.lineTo(X(a),H);cx.stroke();cx.fillText(a+'E',X(a)+3,H-6)}
 for(let a=n0;a<n1;a+=g){cx.beginPath();cx.moveTo(0,Y(a));cx.lineTo(W,Y(a));cx.stroke();cx.fillText(a+'N',4,Y(a)-3)}
 cx.fillStyle='#8590a3';cx.fillText('N ↑',W-50,20*devicePixelRatio);
 if(st){st.drones.forEach((d,i)=>{if(!trails[i])trails[i]=[];if(d.air){trails[i].push([d.n,d.e]);if(trails[i].length>400)trails[i].shift()}});
  trails.forEach(t=>{cx.strokeStyle='rgba(61,220,132,.25)';cx.beginPath();t.forEach((p,j)=>{j?cx.lineTo(X(p[1]),Y(p[0])):cx.moveTo(X(p[1]),Y(p[0]))});cx.stroke()});
  st.drones.forEach(d=>{if(!d.air&&!d.fail){}
   if(d.air){cx.strokeStyle='#aab';cx.lineWidth=2;cx.beginPath();cx.arc(X(d.sp_e),Y(d.sp_n),9*devicePixelRatio,0,7);cx.stroke();
    if(d.m4_n!=null){cx.fillStyle='#ff5a5f';cx.beginPath();cx.arc(X(d.m4_e),Y(d.m4_n),5*devicePixelRatio,0,7);cx.fill();cx.strokeStyle='rgba(255,90,95,.5)';cx.beginPath();cx.moveTo(X(d.e),Y(d.n));cx.lineTo(X(d.m4_e),Y(d.m4_n));cx.stroke()}
    if(d.mo_n!=null){cx.fillStyle='#4da3ff';cx.beginPath();cx.arc(X(d.mo_e),Y(d.mo_n),4*devicePixelRatio,0,7);cx.fill()}}
   cx.fillStyle=d.fail?'#e55':(d.air?'#3ddc84':'#6b7385');cx.beginPath();cx.arc(X(d.e),Y(d.n),(d.air?7:5)*devicePixelRatio,0,7);cx.fill();
   cx.fillStyle='#000';cx.font='bold '+(10*devicePixelRatio)+'px sans-serif';cx.fillText(d.id,X(d.e)-3*devicePixelRatio,Y(d.n)+3.5*devicePixelRatio);cx.lineWidth=1})}
 requestAnimationFrame(draw)}
poll();draw();
</script></body></html>"""


class LiveSwarm:
    def __init__(self, n: int):
        self.n = n
        self.c = SwarmConsole(n=n, use_gazebo=False, force_restart=True)
        self.phase = "starting SITL..."
        self.models4 = [None] * n
        self.modelso = [None] * n
        self.pred = [None] * n
        self.err_hist4, self.err_histo = [], []
        self.lock = threading.Lock()
        self.cmd_busy = False

    def startup(self):
        try:
            self.phase = "launching SITL instances..."
            self.c.launch_sitl()
            self.phase = "connecting MAVLink and waiting for EKF..."
            self.c.connect_all()
            self.c.sync_frames()
            threading.Thread(target=self.c.guidance_loop, daemon=True).start()
            threading.Thread(target=self.predictor, daemon=True).start()
            self.phase = "READY - press Launch swarm"
        except Exception as exc:  # report in the page instead of dying silently
            self.phase = f"ERROR: {exc}"
            print(self.phase, flush=True)

    def predictor(self):
        """Drive both vehicle models in real time with the same setpoints the console sends to ArduPilot."""
        prev = time.time()
        while True:
            time.sleep(0.05)
            now = time.time()
            dt = min(now - prev, 0.2)
            prev = now
            c = self.c
            with c.lock:
                sp = c.centroid_pos[None, :2] + c.current_offsets[:, :2]
                pos = [p.copy() for p in c.last_pos]
                air = [c._active(i) for i in range(self.n)]
            for i in range(self.n):
                if not air[i]:
                    self.models4[i] = self.modelso[i] = None
                    continue
                if self.models4[i] is None:
                    self.models4[i] = AutopilotTrackingModel(pos[i][:2])
                    self.modelso[i] = Drone(drone_id=i, initial_position=pos[i][:2].copy(), max_speed=3.0, max_accel=2.5,
                                            attitude_tau=0.992, drag_coeff=0.637, measurement_noise_std=0.0)
                self.models4[i].step(dt, sp[i])
                d = self.modelso[i]
                d.set_control_input(1.8 * (sp[i] - d.position) + 2.2 * (0.0 - d.velocity))
                d.step(dt)

    def state(self):
        c = self.c
        with c.lock:
            sp = c.centroid_pos[None, :2] + c.current_offsets[:, :2]
            pos = [p.copy() for p in c.last_pos]
            air = [c._active(i) for i in range(self.n)]
            fail = list(c.simulated_faults)
            leader = c.current_leader_idx
            style = c.current_formation_style
        drones, e4s, eos = [], [], []
        for i in range(self.n):
            m4, mo = self.models4[i], self.modelso[i]
            e4 = float(np.linalg.norm(m4.p - pos[i][:2])) * 100 if m4 is not None else None
            eo = float(np.linalg.norm(mo.position - pos[i][:2])) * 100 if mo is not None else None
            if e4 is not None:
                e4s.append(e4)
                eos.append(eo)
            drones.append({
                "id": i + 1, "n": float(pos[i][0]), "e": float(pos[i][1]), "alt": float(-pos[i][2]), "air": bool(air[i]),
                "fail": bool(fail[i]), "leader": bool(i == leader and air[i]), "sp_n": float(sp[i, 0]), "sp_e": float(sp[i, 1]),
                "m4_n": None if m4 is None else float(m4.p[0]), "m4_e": None if m4 is None else float(m4.p[1]),
                "mo_n": None if mo is None else float(mo.position[0]), "mo_e": None if mo is None else float(mo.position[1]),
                "e4": e4, "eo": eo,
            })
        if e4s:
            self.err_hist4.append(float(np.mean(e4s)))
            self.err_histo.append(float(np.mean(eos)))
            self.err_hist4, self.err_histo = self.err_hist4[-1500:], self.err_histo[-1500:]
        return {
            "phase": self.phase, "style": style, "n": self.n, "airborne": int(sum(air)), "drones": drones,
            "err4": float(np.mean(e4s)) if e4s else 0.0, "erro": float(np.mean(eos)) if eos else 0.0,
            "avg4": float(np.mean(self.err_hist4)) if self.err_hist4 else 0.0, "avgo": float(np.mean(self.err_histo)) if self.err_histo else 0.0,
        }

    def command(self, text: str) -> str:
        if not self.phase.startswith("READY") and "ERROR" not in self.phase:
            return "not ready yet"
        if self.cmd_busy:
            return "busy"

        def run():
            self.cmd_busy = True
            try:
                handle_command(self.c, text)
            except Exception as exc:
                print(f"command failed: {exc}", flush=True)
            finally:
                self.cmd_busy = False

        threading.Thread(target=run, daemon=True).start()
        return "sent"


def make_handler(swarm: LiveSwarm):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype="text/html; charset=utf-8", code=200):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/state":
                self._send(json.dumps(swarm.state()), "application/json")
            else:
                self._send(PAGE)

        def do_POST(self):
            if self.path == "/cmd":
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                self._send(swarm.command(str(body.get("c", ""))), "text/plain")
            else:
                self._send("not found", "text/plain", 404)

    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drones", type=int, default=5)
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    swarm = LiveSwarm(a.drones)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(swarm))
    threading.Thread(target=swarm.startup, daemon=True).start()
    print(f"live view on http://127.0.0.1:{a.port} (tunnel with: ssh -L {a.port}:localhost:{a.port} ...)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            swarm.c.land_all()
        except Exception:
            pass
        swarm.c.cleanup()


if __name__ == "__main__":
    main()
