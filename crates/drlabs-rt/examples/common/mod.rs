//! Shared by the bench examples (`rt_bench`, `wave`): the live monitor page,
//! the serial bus factory, the per-axis dump and the Enter-to-stop thread.
#![allow(dead_code)]

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use drlabs_core::lowlevel::LowState;

/// What the monitor page shows, refreshed by the client loop.
#[derive(Clone, Default)]
pub struct Snapshot {
    pub t: f64,
    pub tick: u64,
    pub rate: f64,
    pub miss: u32,
    pub late_max_ms: f64,
    pub estop: u16,
    pub legs: u64,
    pub plan: String,
    pub axes: Vec<AxisSnap>,
}

#[derive(Clone, Default)]
pub struct AxisSnap {
    pub name: String,
    pub q: f32,
    pub q_cmd: f32,
    pub dq: f32,
    pub err_max: f32,
    pub err_max_minute: f32,
    pub voltage: f32,
    pub temp: f32,
    pub status: u8,
    pub blocked: u8,
    pub lo: f32,
    pub hi: f32,
}

/// A flag set when a line arrives on stdin (Enter). A closed or redirected
/// stdin returns 0 bytes at once and never sets it, so a background run must
/// be ended by its timer: an endless run (`seconds == 0`) without a terminal
/// is refused here, before any torque, because the only way left to end it
/// would be killing the process. Ctrl+C bypasses the caller's release path.
pub fn stop_on_enter(seconds: u64) -> Arc<AtomicBool> {
    use std::io::IsTerminal;
    if seconds == 0 && !std::io::stdin().is_terminal() {
        eprintln!("seconds=0 runs until Enter, but stdin is not a terminal; give seconds=N");
        std::process::exit(2);
    }
    let stop = Arc::new(AtomicBool::new(false));
    let flag = Arc::clone(&stop);
    std::thread::spawn(move || {
        let mut line = String::new();
        if matches!(std::io::stdin().read_line(&mut line), Ok(bytes) if bytes > 0) {
            flag.store(true, Ordering::Relaxed);
        }
    });
    stop
}

pub fn dump_axes(state: &LowState, q_cmd: &[f32]) {
    for (i, s) in state.active().iter().enumerate() {
        println!(
            "  j{i:02} q={:+.3} cmd={:+.3} dq={:+.3} I={:+.3} A  V={:.1}  T={:.0} C  fault=0x{:x} status={} blocked={}",
            s.q, q_cmd.get(i).copied().unwrap_or(f32::NAN), s.dq, s.current, s.voltage, s.temp, s.fault, s.status, s.blocked
        );
    }
}

/// A one-file HTTP server: `/` is the page, `/state` the JSON it polls.
/// `title` names the example in the page header and tab.
pub fn serve(port: u16, snapshot: Arc<Mutex<Snapshot>>, title: &'static str) {
    let page = PAGE.replace("{{TITLE}}", title);
    let listener = match TcpListener::bind(("127.0.0.1", port)) {
        Ok(listener) => listener,
        Err(error) => {
            eprintln!("monitor: cannot bind port {port}: {error}");
            return;
        }
    };
    for stream in listener.incoming() {
        let Ok(mut stream) = stream else { continue };
        let _ = stream.set_read_timeout(Some(Duration::from_millis(500)));
        let mut buf = [0u8; 2048];
        let n = stream.read(&mut buf).unwrap_or(0);
        let request = String::from_utf8_lossy(&buf[..n]);
        let path = request.split_whitespace().nth(1).unwrap_or("/");
        let (content_type, body) = if path.starts_with("/state") {
            ("application/json", state_json(&snapshot))
        } else {
            ("text/html; charset=utf-8", page.clone())
        };
        let response = format!(
            "HTTP/1.1 200 OK\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n{body}",
            body.len()
        );
        let _ = stream.write_all(response.as_bytes());
    }
}

fn num(v: f32) -> String {
    if v.is_finite() {
        format!("{v:.4}")
    } else {
        "null".into()
    }
}

fn state_json(snapshot: &Arc<Mutex<Snapshot>>) -> String {
    let snap = snapshot.lock().map(|s| s.clone()).unwrap_or_default();
    let axes: Vec<String> = snap
        .axes
        .iter()
        .map(|a| {
            format!(
                "{{\"name\":\"{}\",\"q\":{},\"q_cmd\":{},\"dq\":{},\"err_max\":{},\"err_max_minute\":{},\"voltage\":{},\"temp\":{},\"status\":{},\"blocked\":{},\"lo\":{},\"hi\":{}}}",
                a.name, num(a.q), num(a.q_cmd), num(a.dq), num(a.err_max), num(a.err_max_minute),
                num(a.voltage), num(a.temp), a.status, a.blocked, num(a.lo), num(a.hi)
            )
        })
        .collect();
    format!(
        "{{\"t\":{:.1},\"tick\":{},\"rate\":{:.1},\"miss\":{},\"late_max_ms\":{:.2},\"estop\":{},\"legs\":{},\"plan\":\"{}\",\"axes\":[{}]}}",
        snap.t, snap.tick, snap.rate, snap.miss, snap.late_max_ms, snap.estop, snap.legs, snap.plan,
        axes.join(",")
    )
}

const PAGE: &str = r##"<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>{{TITLE}} monitor</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
:root{--bg:#0f1115;--fg:#e6e6e6;--mut:#8a93a3;--ok:#3fb950;--bad:#f85149;--warn:#d29922;--cmd:#58a6ff;--q:#f0883e;--line:#262b33}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.4 ui-monospace,Consolas,monospace}
header{padding:12px 16px;border-bottom:1px solid var(--line);display:flex;gap:24px;flex-wrap:wrap;align-items:baseline}
header b{font-size:18px}
.k{color:var(--mut)} .ok{color:var(--ok)} .bad{color:var(--bad)} .warn{color:var(--warn)}
table{border-collapse:collapse;width:100%}
th,td{padding:4px 10px;text-align:right;border-bottom:1px solid var(--line);white-space:nowrap}
th:first-child,td:first-child{text-align:left}
th{color:var(--mut);font-weight:normal}
.bar{position:relative;height:12px;background:#1b1f27;border-radius:3px;min-width:160px}
.bar i{position:absolute;top:2px;width:3px;height:8px;border-radius:1px}
.bar .q{background:var(--q)} .bar .c{background:var(--cmd)}
canvas{display:block;width:100%;height:220px;border-top:1px solid var(--line)}
footer{padding:6px 16px;color:var(--mut)}
</style></head><body>
<header><b>{{TITLE}}</b><span id="plan" class="k"></span><span>t <span id="t">–</span> s</span><span>rate <span id="rate">–</span> Hz</span><span>tick <span id="tick">–</span></span><span>miss <span id="miss">–</span></span><span>late_max <span id="late">–</span> ms</span><span>legs <span id="legs">–</span></span><span id="estop"></span></header>
<table><thead><tr><th>joint</th><th>status</th><th style="text-align:left">range ─ <span style="color:var(--cmd)">■</span> setpoint <span style="color:var(--q)">■</span> position</th><th>q rad</th><th>setpoint</th><th>err</th><th>err max (min)</th><th>err max (run)</th><th>dq rad/s</th><th>V</th><th>°C</th></tr></thead><tbody id="rows"></tbody></table>
<canvas id="chart" width="1200" height="220"></canvas>
<footer>position (orange) and setpoint (blue) of every joint, last 30 s; polled from /state every 100 ms</footer>
<script>
const STATUS={0:"ok",1:"silent",2:"alert",3:"refused",4:"off",5:"tripped"};
const hist=[]; const SPAN=30;
function f(v,d){return (v===null||v===undefined||Number.isNaN(v))?"–":v.toFixed(d)}
async function poll(){
  try{
    const r=await fetch("/state",{cache:"no-store"}); const s=await r.json();
    document.getElementById("plan").textContent=s.plan;
    document.getElementById("t").textContent=f(s.t,0);
    document.getElementById("rate").textContent=f(s.rate,1);
    document.getElementById("tick").textContent=s.tick;
    document.getElementById("miss").textContent=s.miss;
    document.getElementById("late").textContent=f(s.late_max_ms,2);
    document.getElementById("legs").textContent=s.legs;
    const es=document.getElementById("estop"); es.textContent=s.estop?("E-STOP code "+s.estop):"running"; es.className=s.estop?"bad":"ok";
    const rows=document.getElementById("rows"); rows.innerHTML="";
    for(const a of s.axes){
      const tr=document.createElement("tr");
      const err=(a.q!==null&&a.q_cmd!==null)?Math.abs(a.q-a.q_cmd):null;
      const span=(a.hi-a.lo)||1; const pq=a.q===null?0:Math.min(100,Math.max(0,(a.q-a.lo)/span*100)); const pc=a.q_cmd===null?0:Math.min(100,Math.max(0,(a.q_cmd-a.lo)/span*100));
      const st=STATUS[a.status]||a.status; const cls=a.status===0?"ok":(a.status===5||a.status===1?"bad":"warn");
      tr.innerHTML=`<td>${a.name}</td><td class="${cls}">${st}${a.blocked?" ("+a.blocked+")":""}</td><td><div class="bar"><i class="c" style="left:${pc}%"></i><i class="q" style="left:${pq}%"></i></div></td><td>${f(a.q,3)}</td><td>${f(a.q_cmd,3)}</td><td class="${err>0.1?"warn":""}">${f(err,3)}</td><td>${f(a.err_max_minute,3)}</td><td>${f(a.err_max,3)}</td><td>${f(a.dq,2)}</td><td>${f(a.voltage,1)}</td><td>${f(a.temp,0)}</td>`;
      rows.appendChild(tr);
    }
    const now=performance.now()/1000; hist.push({t:now,q:s.axes.map(a=>a.q),c:s.axes.map(a=>a.q_cmd)});
    while(hist.length&&now-hist[0].t>SPAN) hist.shift();
    draw(now);
  }catch(e){ document.getElementById("estop").textContent="no data: "+e; document.getElementById("estop").className="bad"; }
}
function draw(now){
  const cv=document.getElementById("chart"); cv.width=cv.clientWidth; const ctx=cv.getContext("2d"); const W=cv.width,H=cv.height;
  ctx.fillStyle="#0f1115"; ctx.fillRect(0,0,W,H);
  let lo=Infinity,hi=-Infinity; for(const h of hist){for(const v of h.q.concat(h.c)){if(v!==null&&isFinite(v)){lo=Math.min(lo,v);hi=Math.max(hi,v);}}}
  if(!isFinite(lo)) return; if(hi-lo<0.1){const m=(hi+lo)/2;lo=m-0.05;hi=m+0.05;}
  const x=t=>W-(now-t)/SPAN*W, y=v=>H-(v-lo)/(hi-lo)*(H-10)-5;
  ctx.strokeStyle="#262b33"; ctx.beginPath(); for(let s=0;s<=SPAN;s+=5){ctx.moveTo(x(now-s),0);ctx.lineTo(x(now-s),H);} ctx.stroke();
  const n=hist.length?hist[0].q.length:0;
  for(let i=0;i<n;i++){
    for(const [key,col] of [["c","#58a6ff"],["q","#f0883e"]]){
      ctx.strokeStyle=col; ctx.lineWidth=key==="c"?1:1.5; ctx.beginPath(); let pen=false;
      for(const h of hist){const v=h[key][i]; if(v===null||!isFinite(v)){pen=false;continue;} if(!pen){ctx.moveTo(x(h.t),y(v));pen=true;}else ctx.lineTo(x(h.t),y(v));}
      ctx.stroke();
    }
  }
  ctx.fillStyle="#8a93a3"; ctx.fillText(hi.toFixed(2)+" rad",4,12); ctx.fillText(lo.toFixed(2)+" rad",4,H-4);
}
setInterval(poll,100); poll();
</script></body></html>"##;

/// The serial factory of `apps/drlabs-cli/src/machine.rs`, copied: a serial
/// bus carries Dynamixel joints only.
pub fn factory(
    bus: &drlabs_rt::BusConfig,
    joints: &[&drlabs_rt::JointConfig],
) -> Result<Box<dyn drlabs_core::Bus>, String> {
    match bus.transport.as_str() {
        "sim" => drlabs_rt::daemon::sim_factory(bus, joints),
        "serial" => {
            if let Some(joint) = joints.iter().find(|joint| joint.driver != "dynamixel") {
                return Err(format!(
                    "bus {:?}: joint {:?} is a {:?}; a serial bus carries dynamixel joints only",
                    bus.name, joint.name, joint.driver
                ));
            }
            if bus.port.is_empty() {
                return Err(format!("bus {:?}: a serial bus needs a port", bus.name));
            }
            let baud = if bus.baud == 0 { 1_000_000 } else { bus.baud };
            let port = drlabs_transport::serial::open(&bus.port, baud)
                .map_err(|error| format!("bus {:?}: open {}: {error}", bus.name, bus.port))?;
            let shared = drlabs_transport::serial::SharedPort::new(Box::new(port));
            let mut servos = Vec::with_capacity(joints.len());
            for joint in joints {
                let profile = match joint.options.get("profile") {
                    Some(name) => Some(drlabs_dynamixel::profile::profile_named(name).ok_or_else(
                        || format!("joint {:?}: unknown profile {name:?}", joint.name),
                    )?),
                    None => None,
                };
                servos.push((joint.id as u8, profile));
            }
            let period = Duration::from_secs_f64(1.0 / bus.rate_hz.max(1) as f64);
            Ok(Box::new(
                drlabs_dynamixel::DynamixelBus::new(shared, &servos)
                    .with_port_name(&bus.port, baud)
                    .with_period(period),
            ))
        }
        other => Err(format!(
            "bus {:?}: transport {other:?} is not carried by this bench",
            bus.name
        )),
    }
}
