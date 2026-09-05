#!/usr/bin/env python3
"""A monitoring page for the show: every joint, ten minutes of history, the
events, the camera. Standard library only, so it runs wherever the show runs.

demo_show.py starts `Monitor` in a thread; it polls the panel's /api/state
five times a second on its own, independent of the act loop, and keeps
serving while the panel is being restarted (those samples say so). The show
pushes its status and its events into it.

    GET  /                       the page
    GET  /api/snapshot           latest joints, show status, tracker, last events
    GET  /api/history?seconds=N  time series (position error, voltage, e-stop)
    GET  /camera.jpg             the tracker's annotated frame, when there is one
    POST /api/quit               ask the show to finish cleanly (base, release, exit)
"""

import collections
import json
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HISTORY_SECONDS = 600
POLL_HZ = 5.0


class Monitor:
    def __init__(self, panel_url, addr=("127.0.0.1", 8086), history_seconds=HISTORY_SECONDS):
        self.panel_url = panel_url.rstrip("/")
        self.addr = addr
        self.lock = threading.Lock()
        self.samples = collections.deque(maxlen=int(history_seconds * POLL_HZ) + 10)
        self.latest = None            # the last /api/state, or None when the panel is down
        self.latest_at = 0.0
        self.panel_error = "not polled yet"
        self.status = {}
        self.events = collections.deque(maxlen=500)
        self.quit_requested = False
        self.camera = None            # callable -> jpeg bytes or None
        self.started_at = time.time()
        self._server = None

    # -- what the show pushes in

    def set_status(self, **fields):
        with self.lock:
            self.status.update(fields)

    def event(self, kind, text):
        with self.lock:
            self.events.append({"t": time.time(), "kind": kind, "text": text})

    # -- polling the panel

    def _poll_once(self):
        try:
            with urllib.request.urlopen(self.panel_url + "/api/state", timeout=1.5) as r:
                state = json.loads(r.read().decode())
            error = ""
        except Exception as e:  # any transport or parse failure: the panel is down
            state, error = None, str(e)
        now = time.time()
        with self.lock:
            self.latest = state
            self.latest_at = now
            self.panel_error = error
            if state is None:
                self.samples.append({"t": now, "down": True})
                return
            row = {"t": now, "down": False, "joints": {}}
            for j in state["joints"]:
                pos, tgt = j.get("position_rad"), j.get("target_rad")
                row["joints"][j["name"]] = {
                    "err": None if pos is None or tgt is None else abs(pos - tgt),
                    "v": j.get("voltage_v"),
                    "estop": bool(j.get("estop")),
                    "armed": bool(j.get("armed")),
                }
            self.samples.append(row)

    def _poll_loop(self):
        period = 1.0 / POLL_HZ
        while True:
            t0 = time.monotonic()
            self._poll_once()
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # -- what the page reads

    def snapshot(self):
        with self.lock:
            joints = []
            if self.latest is not None:
                for j in self.latest["joints"]:
                    pos, tgt = j.get("position_rad"), j.get("target_rad")
                    joints.append({
                        "name": j["name"], "id": j["id"],
                        "position": pos, "target": tgt,
                        "err": None if pos is None or tgt is None else abs(pos - tgt),
                        "voltage": j.get("voltage_v"), "rate": j.get("rate_hz"),
                        "armed": j.get("armed"), "estop": j.get("estop"),
                        "estop_reason": j.get("estop_reason", ""), "fault": j.get("fault_name", ""),
                        "blocked": j.get("blocked", []), "mode": j.get("mode"),
                    })
            return {
                "now": time.time(),
                "uptime_s": time.time() - self.started_at,
                "panel": {"up": self.latest is not None, "error": self.panel_error,
                          "age_s": time.time() - self.latest_at if self.latest_at else None,
                          "rate_hz": self.latest.get("rate_hz") if self.latest else None,
                          "robot": self.latest.get("robot") if self.latest else None},
                "joints": joints,
                "status": dict(self.status),
                "events": list(self.events)[-60:],
            }

    def history(self, seconds):
        cutoff = time.time() - seconds
        with self.lock:
            rows = [s for s in self.samples if s["t"] >= cutoff]
        # Keep the page light: at most ~600 points per series.
        stride = max(1, len(rows) // 600)
        rows = rows[::stride]
        names = []
        for r in rows:
            if not r["down"]:
                names = list(r["joints"])
                break
        series = {n: {"err": [], "v": [], "estop": []} for n in names}
        t = []
        down = []
        for r in rows:
            t.append(r["t"])
            down.append(r["down"])
            for n in names:
                j = None if r["down"] else r["joints"].get(n)
                series[n]["err"].append(None if j is None else j["err"])
                series[n]["v"].append(None if j is None else j["v"])
                series[n]["estop"].append(False if j is None else j["estop"])
        return {"t": t, "down": down, "series": series}

    # -- serving

    def start(self):
        threading.Thread(target=self._poll_loop, name="monitor-poll", daemon=True).start()
        monitor = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # quiet
                pass

            def _send(self, code, body, ctype="application/json"):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                url = urlparse(self.path)
                if url.path == "/":
                    self._send(200, PAGE.encode(), "text/html; charset=utf-8")
                elif url.path == "/api/snapshot":
                    self._send(200, json.dumps(monitor.snapshot()).encode())
                elif url.path == "/api/history":
                    q = parse_qs(url.query)
                    seconds = float(q.get("seconds", [HISTORY_SECONDS])[0])
                    self._send(200, json.dumps(monitor.history(min(seconds, HISTORY_SECONDS))).encode())
                elif url.path == "/camera.jpg":
                    data = monitor.camera() if monitor.camera else None
                    if data:
                        self._send(200, data, "image/jpeg")
                    else:
                        self._send(404, b"no camera", "text/plain")
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):
                if self.path == "/api/quit":
                    monitor.quit_requested = True
                    monitor.event("quit", "quit requested from the monitor page")
                    self._send(200, b'{"ok":true}')
                else:
                    self._send(404, b"not found", "text/plain")

        self._server = ThreadingHTTPServer(self.addr, Handler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, name="monitor-http", daemon=True).start()
        return f"http://{self.addr[0]}:{self.addr[1]}"


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>UBP show monitor</title>
<style>
  :root { --bg:#111418; --fg:#e6e6e6; --muted:#8a919a; --ok:#3fb950; --warn:#d29922; --bad:#f85149; --line:#2a2f36; }
  body { margin:0; font:13px/1.4 system-ui, sans-serif; background:var(--bg); color:var(--fg); }
  header { display:flex; gap:24px; align-items:center; padding:10px 16px; border-bottom:1px solid var(--line); flex-wrap:wrap; }
  header h1 { font-size:16px; margin:0 12px 0 0; }
  .pill { padding:2px 8px; border-radius:10px; background:#2a2f36; font-size:12px; }
  .pill.ok { background:#1f4d2b; } .pill.bad { background:#5a1d1d; } .pill.warn { background:#5a4a1d; }
  main { display:grid; grid-template-columns: 1fr 1fr; gap:12px; padding:12px 16px; }
  section { background:#171b21; border:1px solid var(--line); border-radius:6px; padding:10px; min-width:0; }
  section h2 { font-size:13px; margin:0 0 8px; color:var(--muted); font-weight:600; }
  table { border-collapse:collapse; width:100%; font-variant-numeric:tabular-nums; }
  th, td { text-align:right; padding:2px 6px; border-bottom:1px solid var(--line); white-space:nowrap; }
  th:first-child, td:first-child { text-align:left; }
  tr.estop td { color:var(--bad); } td.warn { color:var(--warn); } td.bad { color:var(--bad); }
  canvas { width:100%; height:180px; display:block; background:#0d1014; border-radius:4px; }
  #events { max-height:260px; overflow:auto; font-family:ui-monospace, monospace; font-size:12px; }
  #events div { padding:1px 0; border-bottom:1px solid var(--line); }
  .k-estop, .k-recovery { color:var(--bad); } .k-greet { color:var(--ok); } .k-act { color:var(--muted); }
  img { max-width:100%; border-radius:4px; background:#000; }
  button { background:#5a1d1d; color:#fff; border:0; padding:4px 10px; border-radius:4px; cursor:pointer; }
  .wide { grid-column: 1 / -1; }
  .legend { display:flex; flex-wrap:wrap; gap:6px 10px; font-size:11px; color:var(--muted); margin-top:4px; }
  .legend span::before { content:""; display:inline-block; width:10px; height:3px; margin-right:4px; vertical-align:middle; background:currentColor; }
</style></head><body>
<header>
  <h1>UBP show</h1>
  <span id="panel" class="pill">panel</span>
  <span id="show" class="pill">show</span>
  <span id="tracker" class="pill">tracker</span>
  <span id="clock" class="pill"></span>
  <button id="quit" title="base pose, release, exit">Quit show</button>
</header>
<main>
  <section class="wide"><h2>Joints</h2>
    <table id="joints"><thead><tr><th>joint</th><th>id</th><th>pos rad</th><th>target</th><th>err</th><th>V</th><th>Hz</th><th>state</th><th>note</th></tr></thead><tbody></tbody></table>
  </section>
  <section><h2>Voltage (V), last 10 min</h2><canvas id="cv"></canvas><div id="lv" class="legend"></div></section>
  <section><h2>Position error (rad), last 10 min</h2><canvas id="ce"></canvas><div id="le" class="legend"></div></section>
  <section><h2>Events</h2><div id="events"></div></section>
  <section><h2>Camera</h2><img id="cam" alt="no camera"><div id="faces" class="legend"></div></section>
</main>
<script>
const $ = id => document.getElementById(id);
const COLOURS = ["#58a6ff","#3fb950","#d29922","#f85149","#bc8cff","#39c5cf","#f778ba","#ffa657","#7ee787","#a5d6ff","#ffdf5d","#ff7b72","#d2a8ff","#56d4dd","#ffbedd"];
function fmt(x, d) { return x == null ? "—" : Number(x).toFixed(d); }
function hhmmss(t) { return new Date(t * 1000).toLocaleTimeString("ko-KR", {hour12:false}); }

async function snapshot() {
  let s;
  try { s = await (await fetch("/api/snapshot")).json(); } catch (e) { $("panel").textContent = "monitor unreachable"; $("panel").className = "pill bad"; return; }
  const p = s.panel;
  $("panel").textContent = p.up ? `panel up · ${p.robot} · ${p.rate_hz} Hz` : `panel down · ${p.error}`;
  $("panel").className = "pill " + (p.up ? "ok" : "bad");
  const st = s.status || {};
  $("show").textContent = st.act ? `cycle ${st.cycle} · ${st.act} · t=${fmt(st.t,1)} s` + (st.recoveries ? ` · recoveries ${st.recoveries}` : "") + (st.greetings ? ` · greetings ${st.greetings}` : "") : "show: —";
  $("show").className = "pill " + (st.recovering ? "warn" : "ok");
  const tr = st.tracker;
  if (tr) {
    const cand = tr.candidate ? ` · #${tr.candidate.id} ${fmt(tr.candidate.distance_m,2)} m look ${fmt(tr.candidate.looking_for,1)} s` : " · searching";
    $("tracker").textContent = `faces ${tr.faces} · ${tr.fps} fps` + cand + (tr.error ? ` · ${tr.error}` : "");
    $("tracker").className = "pill " + (tr.error ? "bad" : tr.candidate ? "ok" : "");
    $("faces").innerHTML = (tr.tracks || []).map(f => `<span style="color:${f.greeted ? '#f85149' : '#3fb950'}">#${f.id} ${fmt(f.distance_m,2)} m yaw ${fmt(f.yaw,0)}° pit ${fmt(f.pitch,0)}° look ${fmt(f.looking_for,1)} s${f.greeted ? ' greeted' : ''}</span>`).join("");
  } else { $("tracker").textContent = "no tracker"; $("tracker").className = "pill"; }
  $("clock").textContent = `up ${Math.floor(s.uptime_s/60)} min`;
  const tb = $("joints").querySelector("tbody");
  tb.innerHTML = s.joints.map(j => {
    const state = j.estop ? "e-stop" : j.armed ? "armed" : "disarmed";
    const errCls = j.err != null && j.err > 0.2 ? "warn" : "";
    const vCls = j.voltage != null && j.voltage > 8 && j.voltage < 11.0 ? "warn" : (j.voltage != null && j.voltage > 8 && j.voltage < 10.3 ? "bad" : "");
    const note = j.estop ? j.estop_reason : (j.blocked && j.blocked.length ? j.blocked.join(" ") : (j.fault && j.fault !== "No fault" ? j.fault : ""));
    return `<tr class="${j.estop ? 'estop' : ''}"><td>${j.name}</td><td>${j.id}</td><td>${fmt(j.position,3)}</td><td>${fmt(j.target,3)}</td><td class="${errCls}">${fmt(j.err,3)}</td><td class="${vCls}">${fmt(j.voltage,1)}</td><td>${fmt(j.rate,1)}</td><td>${state}</td><td style="text-align:left;max-width:420px;overflow:hidden;text-overflow:ellipsis">${note}</td></tr>`;
  }).join("");
  $("events").innerHTML = s.events.slice().reverse().map(e => `<div class="k-${e.kind}">${hhmmss(e.t)}  ${e.kind}  ${e.text}</div>`).join("");
}

function draw(canvas, legend, hist, key, yMin, yMax, fmtY) {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  canvas.width = W * dpr; canvas.height = H * dpr;
  const ctx = canvas.getContext("2d"); ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, W, H);
  const names = Object.keys(hist.series);
  const t = hist.t; if (t.length < 2) return;
  const t0 = t[0], t1 = t[t.length - 1], span = Math.max(1, t1 - t0);
  const L = 36, R = 6, T = 6, B = 16;
  const x = tt => L + (tt - t0) / span * (W - L - R);
  const y = v => T + (1 - (v - yMin) / (yMax - yMin)) * (H - T - B);
  // panel-down shading and e-stop marks
  ctx.fillStyle = "rgba(248,81,73,0.15)";
  for (let i = 1; i < t.length; i++) if (hist.down[i]) ctx.fillRect(x(t[i-1]), T, Math.max(1, x(t[i]) - x(t[i-1])), H - T - B);
  ctx.strokeStyle = "#2a2f36"; ctx.fillStyle = "#8a919a"; ctx.font = "10px system-ui";
  for (let k = 0; k <= 4; k++) { const v = yMin + (yMax - yMin) * k / 4; ctx.beginPath(); ctx.moveTo(L, y(v)); ctx.lineTo(W - R, y(v)); ctx.stroke(); ctx.fillText(fmtY(v), 2, y(v) + 3); }
  for (let k = 0; k <= 5; k++) { const tt = t0 + span * k / 5; ctx.fillText(hhmmss(tt), Math.max(L, x(tt) - 20), H - 4); }
  names.forEach((n, i) => {
    const s = hist.series[n][key]; ctx.strokeStyle = COLOURS[i % COLOURS.length]; ctx.lineWidth = 1; ctx.beginPath(); let pen = false;
    for (let k = 0; k < s.length; k++) { const v = s[k]; if (v == null || (key === "v" && v < 8)) { pen = false; continue; } const px = x(t[k]), py = y(Math.min(yMax, Math.max(yMin, v))); if (pen) ctx.lineTo(px, py); else ctx.moveTo(px, py); pen = true; }
    ctx.stroke();
  });
  legend.innerHTML = names.map((n, i) => `<span style="color:${COLOURS[i % COLOURS.length]}">${n}</span>`).join("");
}

async function history() {
  let h;
  try { h = await (await fetch("/api/history?seconds=600")).json(); } catch (e) { return; }
  draw($("cv"), $("lv"), h, "v", 9.5, 13.0, v => v.toFixed(1));
  draw($("ce"), $("le"), h, "err", 0, 0.8, v => v.toFixed(1));
}

function camera() { const img = $("cam"); img.src = "/camera.jpg?" + Date.now(); }
$("quit").addEventListener("click", async () => { if (confirm("Finish the show: base pose, release, exit?")) await fetch("/api/quit", {method: "POST"}); });
snapshot(); history(); camera();
setInterval(snapshot, 200); setInterval(history, 2000); setInterval(camera, 200);
</script></body></html>
"""
