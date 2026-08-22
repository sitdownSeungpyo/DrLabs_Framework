// The panel.
//
// Two rules shape everything here.
//
// The first is that a slider is an intent, not a measurement.  Dragging one
// says where the joint should go; where it *is* comes back from the robot and
// is drawn separately.  They are never the same widget, because a control that
// snaps to the measurement hides exactly the disagreement worth seeing.
//
// The second is that nothing safety-related is drawn optimistically.  Armed,
// e-stopped and faulted are rendered only from state the robot sent.  Pressing
// E-STOP does not paint the machine as stopped; the robot saying it is stopped
// does.

const joints = new Map();
let sending = false;
let pending = null;

const messageBox = document.getElementById("message");
const connectionPill = document.getElementById("connection");

function say(text, kind) {
  messageBox.textContent = text || "";
  messageBox.className = "message" + (kind ? " " + kind : "");
}

async function command(payload) {
  const response = await fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const reply = await response.json();
  if (!reply.ok) {
    // A refusal is an answer.  "arm the axis first" is the machine telling
    // the operator something true, and burying it would be the bug.
    say(reply.error || "the robot refused the command", "bad");
  } else {
    say("");
  }
  return reply;
}

// Poses go out at a bounded rate.  A slider drag fires an event per pixel,
// and a robot does not need a new target per pixel — the last one inside the
// window is the one that means anything.
async function flushPose() {
  if (sending || !pending) return;
  sending = true;
  const targets = pending;
  pending = null;
  try {
    await command({ cmd: "pose", joints: targets });
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
  // null is not zero.  A driver that did not read a torque reports null, and
  // "0.00 Nm" would be a different claim entirely.
  if (value === null || value === undefined) return "—";
  return value.toFixed(digits) + (unit ? " " + unit : "");
}

function buildJoints(description) {
  document.getElementById("robot-name").textContent = description.robot;
  document.getElementById("robot-detail").textContent =
    `${description.joints.length} joints · ${description.rate} Hz`;

  const container = document.getElementById("joints");
  container.textContent = "";
  const template = document.getElementById("joint-template");

  for (const spec of description.joints) {
    const node = template.content.cloneNode(true);
    const article = node.querySelector(".joint");
    const slider = node.querySelector(".joint-slider");

    node.querySelector(".joint-name").textContent = spec.name;
    node.querySelector(".joint-driver").textContent =
      `${spec.driver} · ${spec.bus || "—"} · id ${spec.id} · ≤ ${spec.max_torque_nm} Nm`;
    node.querySelector(".joint-min").textContent = spec.min_rad.toFixed(2) + " rad";
    node.querySelector(".joint-max").textContent = spec.max_rad.toFixed(2) + " rad";

    slider.min = spec.min_rad;
    slider.max = spec.max_rad;
    slider.value = 0;

    slider.addEventListener("input", () => {
      entry.commandCell.textContent = number(Number(slider.value), 4, "rad");
      queuePose();
    });

    const entry = {
      spec,
      article,
      slider,
      touched: false,
      armedPill: node.querySelector(".joint-armed"),
      modePill: node.querySelector(".joint-mode"),
      commandCell: node.querySelector(".joint-command"),
      positionCell: node.querySelector(".joint-position"),
      targetCell: node.querySelector(".joint-target"),
      torqueCell: node.querySelector(".joint-torque"),
      rateCell: node.querySelector(".joint-rate"),
      faultCell: node.querySelector(".joint-fault"),
    };
    joints.set(spec.name, entry);
    container.appendChild(node);
  }
}

function render(payload) {
  connectionPill.textContent = payload.connection;
  connectionPill.className =
    "pill " + (payload.connection === "ready" ? "pill-on" : "pill-off");

  const live = payload.connection === "ready";
  for (const state of payload.joints || []) {
    const entry = joints.get(state.name);
    if (!entry) continue;

    // The first state line places the slider where the joint actually is, so
    // that enabling it cannot command a jump.  After that the operator owns
    // it: the panel never drags a control out from under a hand.
    if (!entry.touched && state.position_rad !== null && state.position_rad !== undefined) {
      entry.slider.value = state.position_rad;
      entry.commandCell.textContent = number(state.position_rad, 4, "rad");
      entry.touched = true;
    }
    entry.slider.disabled = !live || !state.armed;

    entry.armedPill.textContent = state.estop
      ? "e-stopped"
      : state.armed
        ? "armed"
        : "disarmed";
    entry.armedPill.className =
      "pill " + (state.estop ? "pill-bad" : state.armed ? "pill-on" : "pill-off");

    // The mode is the difference between an axis that is holding and one
    // that is merely energised.  "Stop (release)" leaves a joint armed and
    // limp, and without this the panel showed that as "armed" next to a stale
    // target — which reads as a joint that should be where it is not.
    entry.modePill.textContent = state.mode || "—";
    entry.modePill.className =
      "pill " + (state.mode === "stop" || !state.mode ? "pill-off" : "pill-on");

    entry.positionCell.textContent = number(state.position_rad, 4, "rad");
    entry.targetCell.textContent = number(state.target_rad, 4, "rad");
    entry.torqueCell.textContent = number(state.torque_nm, 3, "Nm");
    entry.rateCell.textContent = number(state.rate_hz, 0, "Hz");
    entry.faultCell.textContent = state.fault_name || "—";
    entry.article.classList.toggle("faulted", Boolean(state.fault));
  }

  if (!live) {
    for (const entry of joints.values()) {
      entry.slider.disabled = true;
      entry.armedPill.textContent = "no link";
      entry.armedPill.className = "pill pill-off";
      entry.modePill.textContent = "—";
      entry.modePill.className = "pill pill-off";
    }
  }
}

const eventList = document.getElementById("events");
let lastEventSeq = null;

function renderEvents(events) {
  if (!events || !events.length) return;
  const newest = events[events.length - 1];
  const key = newest.joint + ":" + newest.seq;
  if (key === lastEventSeq) return;
  lastEventSeq = key;

  eventList.textContent = "";
  // Newest first: a machine that just tripped should not need scrolling.
  for (const event of events.slice().reverse()) {
    const item = document.createElement("li");

    const level = document.createElement("span");
    level.className = "lvl lvl-" + (event.level || "info");
    level.textContent = event.level || "info";

    const who = document.createElement("span");
    who.className = "who";
    who.textContent = event.joint;

    const what = document.createElement("span");
    what.className = "what";
    what.textContent = event.message;
    if (event.detail) {
      const why = document.createElement("span");
      why.className = "why";
      why.textContent = event.detail;
      what.appendChild(why);
    }

    item.append(level, who, what);
    eventList.appendChild(item);
  }
}

function stream() {
  const source = new EventSource("/api/state");
  source.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      render(payload);
      renderEvents(payload.events);
    } catch (err) {
      /* a malformed frame is not worth tearing the page down for */
    }
  };
  // EventSource reconnects by itself; this only reports the gap.
  source.onerror = () => {
    connectionPill.textContent = "panel offline";
    connectionPill.className = "pill pill-bad";
  };
}

document.getElementById("estop").addEventListener("click", () =>
  command({ cmd: "estop", reason: "web panel" }));
document.getElementById("reset").addEventListener("click", () =>
  command({ cmd: "reset" }));
document.getElementById("stop").addEventListener("click", async () => {
  for (const name of joints.keys()) await command({ cmd: "stop", joint: name });
});
document.getElementById("hold").addEventListener("click", async () => {
  for (const name of joints.keys()) await command({ cmd: "hold", joint: name });
});

fetch("/api/robot")
  .then((response) => response.json())
  .then((description) => {
    buildJoints(description);
    stream();
  })
  .catch(() => say("could not read the robot description", "bad"));
