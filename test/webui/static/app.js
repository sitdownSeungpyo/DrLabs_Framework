'use strict';

// The page is a thin client.  It never decides anything about the machine: it
// posts intents, renders the snapshots that come back, and lets the session
// refuse whatever it wants to refuse.  Every guard that matters lives on the
// other side of these fetches.

const $ = (id) => document.getElementById(id);

let latest = null;        // last status snapshot
let editing = new Set();  // inputs the operator is currently typing into

// --- plumbing --------------------------------------------------------------

async function post(path, body) {
  try {
    const response = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    const result = await response.json();
    if (result.ok === false) {
      banner(result.error, 'err');
    } else if (result.notes && result.notes.length) {
      banner(result.notes.join(' · '), 'warn');
    } else {
      clearBanner();
    }
    return result;
  } catch (err) {
    banner(String(err), 'err');
    return { ok: false };
  }
}

let bannerTimer = null;
function banner(text, kind) {
  const el = $('banner');
  el.textContent = text;
  el.className = 'banner ' + (kind || '');
  clearTimeout(bannerTimer);
  // Errors stay until the next action; anything else clears itself, so the
  // bar does not become permanent furniture.
  if (kind !== 'err') bannerTimer = setTimeout(clearBanner, 4000);
}
function clearBanner() {
  $('banner').className = 'banner hidden';
}

const fmt = (value, digits) =>
  (value === null || value === undefined) ? '—' : Number(value).toFixed(digits);

// Fields the operator is typing into must not be overwritten by the stream.
function track(input) {
  input.addEventListener('focus', () => editing.add(input.id));
  input.addEventListener('blur', () => editing.delete(input.id));
}
function setValue(id, value, digits) {
  if (editing.has(id)) return;
  const el = $(id);
  if (!el) return;
  const text = (digits === undefined) ? value : Number(value).toFixed(digits);
  if (document.activeElement !== el) el.value = text;
}

// --- charts ----------------------------------------------------------------

// A deliberately small plotter: one series, auto-scaled, no dependencies.
// It exists to show shape and timing, not to be a measurement instrument.
function drawChart(canvas, points, colour) {
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (canvas.width !== width * ratio || canvas.height !== height * ratio) {
    canvas.width = width * ratio;
    canvas.height = height * ratio;
  }

  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const values = points.filter((p) => p !== null && p !== undefined);
  if (values.length < 2) return;

  let lo = Math.min(...values);
  let hi = Math.max(...values);
  if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
  const pad = (hi - lo) * 0.12;
  lo -= pad; hi += pad;

  const x = (i) => (i / (points.length - 1)) * width;
  const y = (v) => height - ((v - lo) / (hi - lo)) * height;

  // Zero line, when zero is on screen — it is the reference that makes a
  // velocity or current trace readable at a glance.
  if (lo < 0 && hi > 0) {
    ctx.strokeStyle = 'rgba(139,148,163,0.35)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, y(0));
    ctx.lineTo(width, y(0));
    ctx.stroke();
  }

  ctx.strokeStyle = colour;
  ctx.lineWidth = 1.6;
  ctx.beginPath();
  let started = false;
  points.forEach((value, i) => {
    if (value === null || value === undefined) { started = false; return; }
    if (!started) { ctx.moveTo(x(i), y(value)); started = true; }
    else ctx.lineTo(x(i), y(value));
  });
  ctx.stroke();

  ctx.fillStyle = 'rgba(139,148,163,0.9)';
  ctx.font = '10px ui-monospace, Consolas, monospace';
  ctx.fillText(hi.toFixed(3), 4, 11);
  ctx.fillText(lo.toFixed(3), 4, height - 4);
}

const history = { position: [], velocity: [], current: [] };
const HISTORY = 600;

function pushSamples(samples) {
  for (const sample of samples) {
    history.position.push(sample.position);
    history.velocity.push(sample.velocity);
    history.current.push(sample.current);
  }
  for (const key of Object.keys(history)) {
    if (history[key].length > HISTORY) {
      history[key] = history[key].slice(-HISTORY);
    }
  }
}

// --- rendering -------------------------------------------------------------

function render(status) {
  latest = status;
  const connected = status.connection === 'connected';

  // header
  $('armed-pill').textContent = status.armed ? 'ARMED' : 'DISARMED';
  $('armed-pill').className = 'pill ' + (status.armed ? 'armed' : 'disarmed');
  $('mode-pill').textContent = connected ? status.mode : '—';
  $('rate-pill').textContent =
    connected ? `${status.stats.achieved_hz.toFixed(0)} / ${status.stats.requested_hz} Hz` : '— Hz';

  const link = status.link || {};
  $('link-summary').textContent = connected
    ? Object.entries(link).map(([k, v]) => `${k}=${v}`).join('  ')
    : (status.connect_error || 'not connected');

  // identity
  const id = status.identity;
  $('identity').textContent = id
    ? [`board     ${id.board}`,
       `firmware  ${id.firmware || '—'}`,
       `serial    ${id.serial_number || '—'}`,
       `status    ${id.status}${id.simulated ? '  (simulated)' : ''}`,
       `bounds    ${id.position_min === null ? 'none reported'
          : `[${fmt(id.position_min, 3)}, ${fmt(id.position_max, 3)}] rev`}`].join('\n')
    : '';

  // arming checklist
  const blockers = $('blockers');
  blockers.innerHTML = '';
  for (const item of status.arm_blockers) {
    const div = document.createElement('div');
    div.className = 'blocker';
    div.textContent = '✗ ' + item;
    blockers.appendChild(div);
  }
  const advisories = $('advisories');
  advisories.innerHTML = '';
  for (const item of status.arm_advisories) {
    const div = document.createElement('div');
    div.className = 'advisory';
    div.textContent = '! ' + item;
    advisories.appendChild(div);
  }

  // mode buttons
  for (const button of document.querySelectorAll('#modes button')) {
    button.classList.toggle('active', button.dataset.mode === status.mode);
    button.disabled = !connected;
  }

  // setpoints and their range
  const [lo, hi] = status.effective_limits;
  const posSlider = $('pos-slider');
  posSlider.min = lo; posSlider.max = hi;
  $('pos-range').textContent = `reachable range ${fmt(lo, 3)} … ${fmt(hi, 3)} rev`;
  setValue('pos-slider', status.setpoint.position);
  setValue('pos-input', status.setpoint.position, 3);

  const velSlider = $('vel-slider');
  velSlider.min = -status.safety.max_velocity;
  velSlider.max = status.safety.max_velocity;
  setValue('vel-slider', status.setpoint.velocity);
  setValue('vel-input', status.setpoint.velocity, 3);

  // parameters
  setValue('p-torque', status.params.MaxTorque, 3);
  setValue('p-vel', status.params.VelocityLimit, 3);
  setValue('p-accel', status.params.AccelLimit, 2);
  setValue('p-kp', status.params.KpScale, 2);
  setValue('p-kd', status.params.KdScale, 2);
  setValue('p-ff', status.params.Feedforward, 3);

  // safety
  const sf = status.safety;
  setValue('sf-torque', sf.max_torque, 3);
  setValue('sf-vel', sf.max_velocity, 2);
  setValue('sf-accel', sf.accel_limit, 2);
  setValue('sf-pmin', sf.position_min, 3);
  setValue('sf-pmax', sf.position_max, 3);
  setValue('sf-cur', sf.current_trip, 1);
  setValue('sf-vmin', sf.bus_voltage_min, 1);
  setValue('sf-vmax', sf.bus_voltage_max, 1);
  setValue('sf-btemp', sf.board_temp_trip, 0);
  setValue('sf-mtemp', sf.motor_temp_trip, 0);
  if (!editing.has('sf-poslimit')) $('sf-poslimit').checked = sf.position_limit_enabled;
  if (!editing.has('sf-stall')) $('sf-stall').checked = sf.stall_detect_enabled;
  $('preset-name').textContent = status.preset;
  for (const button of document.querySelectorAll('.presets button')) {
    button.classList.toggle('active', button.dataset.preset === status.preset);
  }

  // telemetry
  const t = status.telemetry;
  $('r-pos').textContent = fmt(t.position, 4);
  $('r-vel').textContent = fmt(t.velocity, 3);
  $('r-torque').textContent = fmt(t.torque, 3);
  $('r-cur').textContent = fmt(t.q_current, 2);
  $('r-volt').textContent = fmt(t.voltage, 1);
  $('r-temp').textContent = fmt(t.temperature, 1);
  $('r-mtemp').textContent = fmt(t.motor_temperature, 1);
  $('r-mode').textContent = t.mode_name || '—';
  $('r-fault').textContent = t.fault > 0 ? `fault ${t.fault} · ${t.fault_name}` : '';

  // trajectory
  const traj = status.trajectory;
  $('traj-bar').style.width = `${(traj.progress * 100).toFixed(1)}%`;
  $('traj-status').textContent = traj.running || traj.progress > 0
    ? [`state     ${traj.running ? 'running' : 'idle'}`,
       `move      ${fmt(traj.start, 4)} → ${fmt(traj.target, 4)} rev`,
       `limits    ${fmt(traj.max_velocity, 2)} rev/s · ${fmt(traj.accel, 2)} rev/s²`,
       `elapsed   ${fmt(traj.elapsed, 2)} / ${fmt(traj.total, 2)} s`,
       `setpoint  ${fmt(traj.setpoint, 4)} rev · feedforward ${fmt(traj.feedforward, 3)} rev/s`].join('\n')
    : '';
  $('traj-start').disabled = !status.armed || traj.running;
  $('traj-stop').disabled = !traj.running;

  // shuttle
  const shuttle = status.shuttle;
  $('shuttle-status').textContent = shuttle.running || shuttle.completed > 0
    ? [`state     ${shuttle.running ? shuttle.phase : 'idle'}`,
       `progress  ${shuttle.status || '—'}`,
       `completed ${shuttle.completed}${shuttle.cycles ? ' / ' + shuttle.cycles : ''} cycles`].join('\n')
    : '';
  $('shuttle-start').disabled = !status.armed || shuttle.running;
  $('shuttle-stop').disabled = !shuttle.running;

  // loop statistics
  const st = status.stats;
  $('loop-stats').textContent = connected
    ? [`rate       ${st.achieved_hz.toFixed(1)} Hz achieved of ${st.requested_hz} requested`,
       `jitter     ${st.jitter_ms.toFixed(2)} ms`,
       `bus        mean ${st.mean_ms.toFixed(2)} ms · p95 ${st.p95_ms.toFixed(2)} ms · max ${st.max_ms.toFixed(2)} ms`,
       `replies    ${(st.reply_rate * 100).toFixed(1)}% of ${st.cycles} cycles · ${st.timeouts} timeouts`,
       `watchdog   ${st.watchdog_ms.toFixed(0)} ms`].join('\n')
    : '';

  // emergency stop banner
  if (status.estop.latched) {
    banner(`EMERGENCY STOP · ${status.estop.reason}` +
           (status.estop.detail ? ` — ${status.estop.detail}` : ''), 'err');
  }

  // buttons that depend on connection state
  $('connect').disabled = connected;
  $('disconnect').disabled = !connected;
  $('arm').disabled = !connected || status.armed;
  $('disarm').disabled = !status.armed;
  $('estop-reset').disabled = !status.estop.latched;
  $('zero').disabled = !connected || status.armed;

  drawChart($('chart-pos'), history.position, '#4da3ff');
  drawChart($('chart-vel'), history.velocity, '#35c07a');
  drawChart($('chart-cur'), history.current, '#e0a33a');
}

function appendLog(events) {
  const log = $('log');
  for (const event of events) {
    const div = document.createElement('div');
    div.className = event.level;
    const time = new Date(event.at).toLocaleTimeString();
    div.innerHTML =
      `<span class="lvl">${event.level.toUpperCase()}</span>` +
      `<span>${time}  ${escapeHTML(event.message)}</span>`;
    log.appendChild(div);
    if (event.detail) {
      const detail = document.createElement('div');
      detail.className = 'detail';
      detail.textContent = event.detail;
      log.appendChild(detail);
    }
  }
  while (log.children.length > 400) log.removeChild(log.firstChild);
  log.scrollTop = log.scrollHeight;
}

function escapeHTML(text) {
  const div = document.createElement('div');
  div.textContent = text;
  return div.innerHTML;
}

// --- stream ----------------------------------------------------------------

function connectStream() {
  const source = new EventSource('/api/stream');
  source.onmessage = (message) => {
    const data = JSON.parse(message.data);
    pushSamples(data.samples || []);
    if (data.events && data.events.length) appendLog(data.events);
    render(data.status);
  };
  source.onerror = () => {
    // EventSource reconnects on its own; say so rather than looking frozen.
    $('link-summary').textContent = 'stream interrupted, reconnecting…';
  };
}

// --- wiring ----------------------------------------------------------------

async function loadLists() {
  const drivers = await (await fetch('/api/drivers')).json();
  const select = $('driver');
  select.innerHTML = '';
  for (const driver of drivers) {
    const option = document.createElement('option');
    option.value = driver.name;
    option.textContent = `${driver.name} — ${driver.description}`;
    select.appendChild(option);
  }
  await refreshPorts();

  const defaults = await (await fetch('/api/defaults')).json();
  if (defaults.driver) select.value = defaults.driver;
  if (defaults.port) $('port').value = defaults.port;
  if (defaults.id) $('id').value = defaults.id;
  if (defaults.model) $('model').value = defaults.model;
  if (defaults.rate_hz) $('rate').value = defaults.rate_hz;
}

async function refreshPorts() {
  const ports = await (await fetch('/api/ports')).json();
  const list = $('ports');
  list.innerHTML = '';
  for (const port of ports) {
    const option = document.createElement('option');
    option.value = port.Path || port.path;
    list.appendChild(option);
  }
}

function wire() {
  for (const input of document.querySelectorAll('input')) track(input);

  $('connect').onclick = () => post('/api/connect', {
    driver: $('driver').value,
    port: $('port').value,
    baud: 1000000,
    id: Number($('id').value),
    model: $('model').value,
    rate_hz: Number($('rate').value),
    allow_unknown_board: $('allow-unknown').checked,
  });
  $('disconnect').onclick = () => post('/api/disconnect');
  $('refresh-ports').onclick = refreshPorts;

  $('arm').onclick = () => post('/api/arm', { acknowledged: $('ack').checked });
  $('disarm').onclick = () => post('/api/disarm');
  $('estop').onclick = () => post('/api/estop');
  $('estop-reset').onclick = () => post('/api/estop/reset');
  $('zero').onclick = () => post('/api/zero');

  for (const button of document.querySelectorAll('#modes button')) {
    button.onclick = () => post('/api/mode', { mode: button.dataset.mode });
  }

  // Slider and number box drive each other, and both send on release rather
  // than on every pixel — a drag should not be a thousand commands.
  const sendPosition = (value) => post('/api/setpoint', { position: Number(value) });
  $('pos-slider').oninput = (e) => { $('pos-input').value = Number(e.target.value).toFixed(3); };
  $('pos-slider').onchange = (e) => sendPosition(e.target.value);
  $('pos-input').onchange = (e) => { $('pos-slider').value = e.target.value; sendPosition(e.target.value); };

  const sendVelocity = (value) => post('/api/setpoint', { velocity: Number(value) });
  $('vel-slider').oninput = (e) => { $('vel-input').value = Number(e.target.value).toFixed(3); };
  $('vel-slider').onchange = (e) => sendVelocity(e.target.value);
  $('vel-input').onchange = (e) => { $('vel-slider').value = e.target.value; sendVelocity(e.target.value); };

  $('apply-params').onclick = () => post('/api/params', {
    MaxTorque: Number($('p-torque').value),
    VelocityLimit: Number($('p-vel').value),
    AccelLimit: Number($('p-accel').value),
    KpScale: Number($('p-kp').value),
    KdScale: Number($('p-kd').value),
    ILimitScale: latest ? latest.params.ILimitScale : 1,
    Feedforward: Number($('p-ff').value),
  });

  $('apply-safety').onclick = () => post('/api/safety', {
    MaxTorque: Number($('sf-torque').value),
    MaxVelocity: Number($('sf-vel').value),
    AccelLimit: Number($('sf-accel').value),
    PositionMin: Number($('sf-pmin').value),
    PositionMax: Number($('sf-pmax').value),
    PositionLimitEnabled: $('sf-poslimit').checked,
    CurrentTrip: Number($('sf-cur').value),
    BusVoltageMin: Number($('sf-vmin').value),
    BusVoltageMax: Number($('sf-vmax').value),
    BoardTempTrip: Number($('sf-btemp').value),
    MotorTempTrip: Number($('sf-mtemp').value),
    StallDetectEnabled: $('sf-stall').checked,
  });

  for (const button of document.querySelectorAll('.presets button')) {
    button.onclick = () => post('/api/preset', { preset: button.dataset.preset });
  }

  $('traj-start').onclick = () => post('/api/trajectory/start', {
    target: Number($('t-target').value),
    max_velocity: Number($('t-vel').value),
    accel: Number($('t-accel').value),
  });
  $('traj-stop').onclick = () => post('/api/trajectory/stop');

  $('shuttle-start').onclick = () => post('/api/shuttle/start', {
    a: Number($('s-a').value),
    b: Number($('s-b').value),
    dwell: Number($('s-dwell').value),
    cycles: Number($('s-cycles').value),
  });
  $('shuttle-stop').onclick = () => post('/api/shuttle/stop');

  // The space bar is the emergency stop.  It is the one control that has to
  // work without the operator finding a button first.
  document.addEventListener('keydown', (event) => {
    if (event.code === 'Space' && event.target.tagName !== 'INPUT') {
      event.preventDefault();
      post('/api/estop');
    }
  });
}

wire();
loadLists().then(connectStream);
