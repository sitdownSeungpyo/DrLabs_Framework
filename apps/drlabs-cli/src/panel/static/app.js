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

// Only the joint being dragged is commanded.
//
// Sending every slider on every drag meant touching one joint restated a target
// for all sixteen. They were mostly the measured positions, so it mostly looked
// like nothing — but "mostly" is the wrong word for a command, and on a chain it
// is fifteen targets nobody asked for.
function queuePose(name, value) {
  pending = Object.assign(pending || {}, { [name]: value });
  flushPose();
}

// A profiled move. Everything that crosses more than a nudge goes through here
// rather than through /api/pose: a setpoint is a step, and a step across sixty
// degrees is the joint going as fast as its limits allow with nothing shaping
// the start or the stop.
function moveVelocity() {
  const value = Number(document.getElementById("move-velocity").value);
  return Number.isFinite(value) && value > 0 ? value : 0.35;
}

async function moveJoints(targets) {
  return post("/api/move", { targets, velocity: moveVelocity() });
}

async function moveToPose(pose) {
  return post("/api/move", { pose, velocity: moveVelocity() });
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
      queuePose(description.name, Number(slider.value));
    });

    node.querySelector(".joint-min").textContent =
      number(description.min_rad, 2, "rad");
    node.querySelector(".joint-max").textContent =
      number(description.max_rad, 2, "rad");

    // Per-joint commanding: a number to go to, and the two moves worth having
    // a button for. Both are profiled, because a joint asked to cross sixty
    // degrees should be shaped on the way.
    const goal = node.querySelector(".joint-goal");
    goal.min = description.min_rad;
    goal.max = description.max_rad;
    goal.value = (description.position_rad ?? 0).toFixed(3);

    const go = node.querySelector(".joint-go");
    go.addEventListener("click", () => {
      const target = Number(goal.value);
      if (!Number.isFinite(target)) {
        say(`${description.name}: ${goal.value || "that"} is not an angle`, "bad");
        return;
      }
      moveJoints({ [description.name]: target });
    });

    node.querySelector(".joint-zero").addEventListener("click", () => {
      goal.value = "0.000";
      moveJoints({ [description.name]: 0 });
    });

    container.appendChild(node);
    joints.set(description.name, { node, slider, goal, touched: false, editing: false });
  }
  buildPoses(state.poses || []);
  built = true;
}

// One button per pose the description carries. The panel does not know what
// "base" means and does not need to — the machine's own description says.
function buildPoses(poses) {
  const container = document.getElementById("poses");
  container.innerHTML = "";
  for (const name of poses) {
    if (name === "zero") continue; // already its own button
    const button = document.createElement("button");
    button.className = "btn btn-move";
    button.textContent = `Go to ${name}`;
    button.addEventListener("click", () => moveToPose(name));
    container.appendChild(button);
  }
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

    const commandable = joint.armed && !joint.estop;
    const goal = entry.goal;
    goal.disabled = !commandable;
    node.querySelector(".joint-go").disabled = !commandable;
    node.querySelector(".joint-zero").disabled = !commandable;

    // The goal box follows the machine until somebody types in it, for the same
    // reason the slider does: overwriting a half-typed number is worse than
    // showing one that is briefly stale.
    if (!entry.editing && joint.position_rad !== null && document.activeElement !== goal) {
      goal.value = joint.position_rad.toFixed(3);
    }

    node.querySelector(".joint-progress").textContent =
      joint.moving_to_rad === null || joint.moving_to_rad === undefined
        ? ""
        : `moving to ${number(joint.moving_to_rad, 3, "rad")}`;

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
document.addEventListener("input", (event) => {
  const box = event.target.closest(".joint-goal");
  if (!box) return;
  for (const [, entry] of joints) {
    if (entry.goal === box) entry.editing = true;
  }
});
document.addEventListener("focusout", (event) => {
  const box = event.target.closest(".joint-goal");
  if (!box) return;
  for (const [, entry] of joints) {
    if (entry.goal === box) entry.editing = false;
  }
});

document.addEventListener("pointerup", () => {
  setTimeout(() => {
    for (const [, entry] of joints) entry.touched = false;
  }, 1500);
});

// Every joint to zero, together and under a profile. The description carries a
// "zero" pose; falling back to naming the joints keeps the button working on a
// description that does not.
document.getElementById("go-zero").addEventListener("click", () => {
  const targets = {};
  for (const name of joints.keys()) targets[name] = 0;
  moveJoints(targets);
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

// ---------------------------------------------------------------------------
// Teach mode.
//
// The limits in a description have to come from somewhere, and on a machine
// nobody has characterised they are a guess. This measures them instead: torque
// off, the operator moves each joint through its travel, and the extremes the
// encoders reported are what comes back.
//
// Nothing here applies itself. The numbers are shown next to what the
// description says today, in fields the operator can correct, and writing them
// back is a separate deliberate press.
// ---------------------------------------------------------------------------

const teachRows = new Map();
let teachBuilt = false;
let teachRunning = false;

function teachSeconds() {
  const value = Number(document.getElementById("learn-seconds").value);
  return Number.isFinite(value) && value > 0 ? value : 60;
}

function buildTeachTable(joints) {
  const container = document.getElementById("teach-results");
  container.innerHTML = "";
  teachRows.clear();

  const table = document.createElement("table");
  table.className = "teach-table";
  table.innerHTML =
    "<thead><tr><th>joint</th><th>seen</th><th>span</th>" +
    "<th>min rad</th><th>max rad</th><th>dir</th><th>in the file</th></tr></thead>";
  const body = document.createElement("tbody");

  const template = document.getElementById("teach-row-template");
  for (const joint of joints) {
    const row = template.content.firstElementChild.cloneNode(true);
    row.querySelector(".teach-name").textContent = joint.name;
    body.appendChild(row);
    teachRows.set(joint.name, {
      row,
      min: row.querySelector(".teach-min"),
      max: row.querySelector(".teach-max"),
      direction: row.querySelector(".teach-direction"),
      // Once the operator corrects a number it is theirs; a later sample must
      // not overwrite it mid-edit.
      edited: false,
    });
    for (const field of ["teach-min", "teach-max"]) {
      row.querySelector("." + field).addEventListener("input", () => {
        teachRows.get(joint.name).edited = true;
      });
    }
  }

  table.appendChild(body);
  container.appendChild(table);

  const apply = document.createElement("button");
  apply.id = "learn-apply";
  apply.className = "btn btn-apply";
  apply.textContent = "Write these limits to the description";
  apply.addEventListener("click", applyLimits);
  container.appendChild(apply);

  const note = document.createElement("p");
  note.className = "muted note";
  note.textContent =
    "Writing keeps the previous file alongside as .json.bak. The running " +
    "machine is not changed — its envelope and its direction were built when " +
    "it came up — so restart the panel to drive against the new limits. " +
    "Flipping a direction inverts that joint's sense, so re-record it after.";
  container.appendChild(note);

  teachBuilt = true;
}

function renderTeach(status) {
  const pill = document.getElementById("teach-state");
  teachRunning = status.running;
  pill.textContent = status.running
    ? `recording · ${status.remaining_s.toFixed(0)} s left`
    : status.elapsed_s > 0
      ? `recorded ${status.elapsed_s.toFixed(0)} s`
      : "idle";
  pill.className = "pill " + (status.running ? "pill-ok" : "pill-off");

  document.getElementById("learn-start").disabled = status.running;
  document.getElementById("learn-stop").disabled = !status.running;
  document.getElementById("learn-clear").disabled =
    status.running || status.elapsed_s === 0;

  const anySamples = status.joints.some((joint) => joint.samples > 0);
  if (!anySamples) {
    if (teachBuilt) {
      document.getElementById("teach-results").innerHTML = "";
      teachRows.clear();
      teachBuilt = false;
    }
    return;
  }
  if (!teachBuilt) buildTeachTable(status.joints);

  for (const joint of status.joints) {
    const entry = teachRows.get(joint.name);
    if (!entry) continue;

    entry.row.querySelector(".teach-seen").textContent =
      joint.samples === 0
        ? "not seen"
        : `${number(joint.min_rad, 3)} … ${number(joint.max_rad, 3)}`;
    entry.row.querySelector(".teach-span").textContent =
      joint.span_rad === null || joint.span_rad === undefined
        ? "—"
        : `${((joint.span_rad * 180) / Math.PI).toFixed(1)}°`;
    entry.row.querySelector(".teach-configured").textContent =
      `${number(joint.configured_min_rad, 3)} … ${number(joint.configured_max_rad, 3)}`;

    if (!entry.edited && joint.samples > 0) {
      entry.min.value = joint.min_rad.toFixed(3);
      entry.max.value = joint.max_rad.toFixed(3);
    }
    if (!entry.edited) entry.direction.value = String(joint.direction);
  }
}

async function applyLimits() {
  const edits = {};
  for (const [name, entry] of teachRows) {
    const min = Number(entry.min.value);
    const max = Number(entry.max.value);
    if (!Number.isFinite(min) || !Number.isFinite(max)) {
      say(`${name}: ${entry.min.value} … ${entry.max.value} is not a range`, "bad");
      return;
    }
    if (max <= min) {
      say(`${name}: max must be above min (${min} … ${max})`, "bad");
      return;
    }
    edits[name] = { min_rad: min, max_rad: max, direction: Number(entry.direction.value) };
  }
  if (!Object.keys(edits).length) {
    say("there is nothing recorded to write", "bad");
    return;
  }

  const response = await fetch("/api/limits", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(edits),
  }).catch(() => null);

  if (!response) {
    say("lost contact with the robot", "bad");
    return;
  }
  const reply = await response.json().catch(() => ({}));
  if (!response.ok) {
    say(reply.error || "the description was not written", "bad");
    return;
  }
  say(`written to ${reply.written} — restart the panel to drive against it`, "ok");
}

// Releasing a fifteen-degree-of-freedom machine drops it, so the button asks
// twice. Deliberately not a native browser dialog: those block the page, and a
// panel whose whole job is to keep showing a live machine must not stop
// rendering because a dialog is open.
let releaseArmed = null;

document.getElementById("learn-start").addEventListener("click", async () => {
  const button = document.getElementById("learn-start");
  if (!releaseArmed) {
    button.textContent = "Press again — the machine goes limp";
    button.classList.add("btn-arming");
    say(
      "This releases every joint. The machine goes limp and will fall under " +
      "its own weight if it is not supported.",
      "bad"
    );
    releaseArmed = setTimeout(() => {
      releaseArmed = null;
      button.textContent = "Release & record";
      button.classList.remove("btn-arming");
      say("");
    }, 5000);
    return;
  }

  clearTimeout(releaseArmed);
  releaseArmed = null;
  button.textContent = "Release & record";
  button.classList.remove("btn-arming");

  teachRows.clear();
  teachBuilt = false;
  document.getElementById("teach-results").innerHTML = "";
  await post("/api/learn/start", { seconds: teachSeconds() });
});
document.getElementById("learn-stop").addEventListener("click", () => post("/api/learn/stop"));
document.getElementById("learn-clear").addEventListener("click", async () => {
  await post("/api/learn/clear");
  teachRows.clear();
  teachBuilt = false;
  document.getElementById("teach-results").innerHTML = "";
});

async function pollTeach() {
  try {
    const response = await fetch("/api/learn");
    renderTeach(await response.json());
  } catch (error) {
    /* the state poll already reports lost contact */
  }
}
pollTeach();
setInterval(pollTeach, 250);
