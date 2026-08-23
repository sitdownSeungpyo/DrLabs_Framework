// The panel.
//
// Two rules shape everything here.
//
// The first is that a slider is an intent, not a measurement. Dragging one says
// where the joint should go; where it *is* comes back from the robot and is
// drawn separately. They are never the same widget, because a control that
// snaps to the measurement hides exactly the disagreement worth seeing.
//
// The second is that nothing safety-related is drawn optimistically. Armed,
// e-stopped and faulted are rendered only from state the robot sent. Pressing
// E-STOP does not paint the machine as stopped; the robot saying it is stopped
// does.

const joints = new Map();
let sending = false;
let pending = null;
let built = false;

const messageBox = document.getElementById("message");
const connectionPill = document.getElementById("connection");

function say(text, kind) {
  messageBox.textContent = text || "";
  messageBox.className = "message" + (kind ? " " + kind : "");
}

async function post(path, payload) {
  try {
    const response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload || {}),
    });
    if (response.ok) {
      say("");
      return true;
    }
    // A refusal is an answer. "the axis is not armed" is the machine telling
    // the operator something true, and burying it would be the bug.
    const reply = await response.json().catch(() => ({}));
    say(reply.error || "the robot refused the command", "bad");
    return false;
  } catch (error) {
    say("lost contact with the robot", "bad");
    return false;
  }
}

// Poses go out at a bounded rate. A slider drag fires an event per pixel, and a
// robot does not need a new target per pixel — the last one inside the window
// is the one that means anything.
//
// The loop underneath enforces its own rate regardless, so this is politeness
// rather than protection. It is still worth doing: a socket carrying a thousand
// identical poses a second is a socket that is not carrying state back.
async function flushPose() {
  if (sending || !pending) return;
  sending = true;
  const targets = pending;
  pending = null;
  try {
    await post("/api/pose", targets);
  } finally {
    sending = false;
    if (pending) setTimeout(flushPose, 0);
  }
}

function queuePose() {
  const targets = {};
  for (const [name, joint] of joints) {
    targets[name] = Number(joint.slider.value);
  }
  pending = targets;
  flushPose();
}

function number(value, digits, unit) {
  // null is not zero. A driver that did not read a torque reports null, and
  // "0.00 Nm" would be a different claim entirely.
  if (value === null || value === undefined) return "—";
  return value.toFixed(digits) + (unit ? " " + unit : "");
}

function buildJoints(state) {
  document.getElementById("robot-name").textContent = state.robot || "robot";
  document.getElementById("robot-detail").textContent =
    `${state.joints.length} joints · ${state.rate_hz} Hz`;

  const container = document.getElementById("joints");
  container.innerHTML = "";
  joints.clear();

  const template = document.getElementById("joint-template");
  for (const description of state.joints) {
    const node = template.content.firstElementChild.cloneNode(true);
    node.querySelector(".joint-name").textContent = description.name;
    node.querySelector(".joint-driver").textContent =
      `${description.driver} · id ${description.id} · ${description.bus}`;

    const slider = node.querySelector(".joint-slider");
    slider.min = description.min_rad;
    slider.max = description.max_rad;
    slider.value = description.position_rad ?? 0;
    slider.disabled = true;
    slider.addEventListener("input", () => {
      node.querySelector(".joint-command").textContent =
        number(Number(slider.value), 3, "rad");
      queuePose();
    });

    node.querySelector(".joint-min").textContent =
      number(description.min_rad, 2, "rad");
    node.querySelector(".joint-max").textContent =
      number(description.max_rad, 2, "rad");

    container.appendChild(node);
    joints.set(description.name, { node, slider, touched: false });
  }
  built = true;
}

function render(state) {
  connectionPill.textContent = state.connected ? "live" : "disconnected";
  connectionPill.className = "pill " + (state.connected ? "pill-ok" : "pill-off");

  for (const joint of state.joints) {
    const entry = joints.get(joint.name);
    if (!entry) continue;
    const { node, slider } = entry;

    // The slider follows the machine only until somebody takes hold of it.
    // After that it is the operator's, and snapping it back to the measurement
    // would fight them mid-drag.
    if (!entry.touched && joint.position_rad !== null) {
      slider.value = joint.position_rad;
      node.querySelector(".joint-command").textContent =
        number(joint.position_rad, 3, "rad");
    }
    slider.disabled = !joint.armed || joint.estop;

    // The marker class has to survive being restyled. Assigning className
    // wholesale drops the very class this element was found by, so the first
    // render works, the second cannot find it, and the panel freezes on stale
    // numbers while still looking alive. Found in a browser, because that is
    // the only place it exists.
    const armed = node.querySelector(".joint-armed");
    armed.textContent = joint.estop ? "e-stopped" : joint.armed ? "armed" : "disarmed";
    armed.className =
      "joint-armed pill " +
      (joint.estop ? "pill-bad" : joint.armed ? "pill-ok" : "pill-off");

    const mode = node.querySelector(".joint-mode");
    mode.textContent = joint.mode;
    mode.className = "joint-mode pill " + (joint.armed ? "pill-ok" : "pill-off");

    node.querySelector(".joint-position").textContent =
      number(joint.position_rad, 4, "rad");
    node.querySelector(".joint-target").textContent =
      number(joint.target_rad, 4, "rad");
    node.querySelector(".joint-torque").textContent =
      number(joint.torque_nm, 3, "Nm");
    node.querySelector(".joint-rate").textContent =
      number(joint.rate_hz, 0, "Hz");

    const note = joint.estop
      ? joint.estop_reason
      : joint.blocked.length
        ? joint.blocked.join(" ")
        : joint.fault_name || "—";
    node.querySelector(".joint-note").textContent = note || "—";
  }
}

async function poll() {
  try {
    const response = await fetch("/api/state");
    const state = await response.json();
    if (!built) buildJoints(state);
    render(state);
  } catch (error) {
    connectionPill.textContent = "disconnected";
    connectionPill.className = "pill pill-off";
  }
}

// The sliders are the operator's once touched, and theirs again after a pause.
document.addEventListener("pointerdown", (event) => {
  const slider = event.target.closest(".joint-slider");
  if (!slider) return;
  for (const [, entry] of joints) {
    if (entry.slider === slider) entry.touched = true;
  }
});
document.addEventListener("pointerup", () => {
  setTimeout(() => {
    for (const [, entry] of joints) entry.touched = false;
  }, 1500);
});

document.getElementById("estop").addEventListener("click", () =>
  post("/api/estop", { reason: "the panel's E-STOP was pressed" }));
document.getElementById("reset").addEventListener("click", () => post("/api/reset"));
document.getElementById("hold").addEventListener("click", () => post("/api/hold"));
document.getElementById("stop").addEventListener("click", () => post("/api/stop"));

// Ten a second is faster than anybody reads and slower than the control loop
// runs, which is the right side of both.
poll();
setInterval(poll, 100);
