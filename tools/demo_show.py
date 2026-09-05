#!/usr/bin/env python3
"""Demo: a playlist of motions, repeated forever, that survives an e-stop.

A throwaway upper-layer program, like demo_circle.py, and built from its
parts: it drives a `drlabs panel` over HTTP, so every setpoint still passes
the panel's envelope, e-stop and velocity limits. It never touches a bus.

    python tools/demo_show.py --launch configs/ubp15.json      (the show owns the panel)
    python tools/demo_show.py                                   (a panel is already on 8085)
    python tools/demo_show.py --dry-run                         (joint ranges per act, no robot)

The playlist is a comma-separated list of acts, each with an optional repeat
count: the default `eight*3,wave,rest` draws three figure-eights with both
arms, waves one hand (alternating arms each time round), rests two seconds,
and starts again.

Acts
----
eight   Both hands trace the same upright figure-eight (a Gerono lissajous),
        each in front of its own shoulder in a forward-up plane — the same
        reachable region as demo_circle's --shared circle, for the same
        reason: the shoulders barely adduct, so the hands cannot meet.
wave    One arm raised forward with the forearm folded up, waving by rotating
        the elbow yaw; the gripper folds once; the head nods. Which arm waves
        alternates each time the act comes round. The pose is entered and
        left inside the act with a smooth ramp from the base pose.
rest    The base pose, held for a moment.

Recovery
--------
When a joint e-stops (a lost reply, a voltage sag, the panel's own button)
the show does not end. With --launch the panel is stopped and started again
— after a communication loss the session tears its link down and only a
fresh bring-up reopens it (crates/drlabs-session/src/control_loop.rs,
"Reconnecting is the operator's move"). Without --launch the show can only
POST /api/reset, which is enough for an e-stop the link survived. Either way
the machine is then brought back to the base pose under a profile and the
interrupted act starts over. Attempts back off 5 → 60 s, so a bus that has
lost power waits quietly until it is powered again.

Sign conventions are demo_circle's (sh_p negative = forward, sh_r negative =
outward, elb_p negative = flexed). Shoulder yaw and elbow yaw signs are not
verified on the robot, so the wave uses elbow yaw symmetrically about zero.
"""

import argparse
import datetime
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from urllib.error import HTTPError, URLError

import demo_circle as dc

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ARM_KEYS = ("sh_p", "sh_r", "elb_p")
PINNED_KEYS = ("sh_y", "elb_y", "wrist_p")


def now():
    return datetime.datetime.now().strftime("%H:%M:%S")


def log(msg):
    print(f"{now()}  {msg}", flush=True)


class Interrupted(Exception):
    """The machine, or the panel, stopped taking the show's commands."""


class Cut(Exception):
    """An act was cut short on purpose: someone is to be greeted, or the
    monitor asked the show to finish. `.reason` says which; `.track` is the
    face for a greeting."""

    def __init__(self, reason, track=None):
        super().__init__(reason)
        self.reason = reason
        self.track = track


# The show's own view of the machine, for the page and the log. None until
# main() builds one; every writer checks.
MONITOR = None


def note(kind, text):
    log(f"{kind}: {text}" if kind not in ("act",) else text)
    if MONITOR is not None:
        MONITOR.event(kind, text)


# ---------------------------------------------------------------- panel

class Client(dc.Panel):
    """demo_circle's client with every transport failure turned into
    Interrupted, so the runner has one thing to catch."""

    def get(self, path):
        try:
            return super().get(path)
        except HTTPError as error:
            raise Interrupted(f"GET {path}: HTTP {error.code} {error.reason}")
        except (URLError, OSError, ValueError) as error:
            raise Interrupted(f"GET {path}: {error}")

    def post(self, path, body):
        try:
            return super().post(path, body)
        except (URLError, OSError, ValueError) as error:
            raise Interrupted(f"POST {path}: {error}")


def stopped_joints(joints, names=None):
    return [f"{n} ({j['estop_reason']})" for n, j in joints.items()
            if j["estop"] and (names is None or n in names)]


# ---------------------------------------------------------------- acts

class Act:
    name = ""
    duration = 0.0

    def targets(self, t):
        raise NotImplementedError

    def sample(self, count=200):
        """Setpoints across the act, for range checks and the dry run."""
        return [self.targets(self.duration * k / count) for k in range(count + 1)]


class Eight(Act):
    """Both arms, an upright figure-eight in front of each shoulder."""

    def __init__(self, args, repeats, base, gripper, head):
        self.name = "eight"
        self.args = args
        self.repeats = repeats
        self.period = args.period
        self.duration = args.period * repeats
        self.base = base
        self.gripper = gripper
        self.head = head
        self.path = self._plan()

    def _plan(self):
        args = self.args
        n = int(args.period * args.rate)
        q = {"l": [0.5, 0.2, 1.2], "r": [0.5, 0.2, 1.2]}
        path = []
        for k in range(n):
            phase = 2 * math.pi * k / n
            # Gerono lissajous, tall axis vertical: the hand goes up the middle,
            # round the top lobe, down the middle, round the bottom lobe.
            dx = args.eight_ax * math.sin(phase) * math.cos(phase)
            dz = args.eight_az * math.sin(phase)
            per_arm = {}
            for side in ("l", "r"):
                sy = dc.SHOULDER[side][1]
                target = dc.base_to_shoulder(side, (args.eight_cx + dx, sy, args.eight_cz + dz))
                q[side], res = dc.ik(target, q[side], args.upper, args.lower)
                if res > 0.003:
                    raise SystemExit(
                        f"eight: IK did not converge at phase {phase:.2f} ({res*1000:.1f} mm); "
                        f"shrink --eight-ax/--eight-az or move --eight-cx/--eight-cz")
                per_arm[side] = dc.to_joint_space(q[side], args.sh_p_sign)
            path.append(per_arm)
        return path

    def targets(self, t):
        n = len(self.path)
        fraction = (t / self.period) % 1.0
        per_arm = self.path[int(fraction * n) % n]
        out = {}
        for side in ("l", "r"):
            for key in ARM_KEYS:
                out[dc.ARMS[side][key]] = per_arm[side][key]
            for key in PINNED_KEYS:
                name = dc.ARMS[side][key]
                if name in self.base:
                    out[name] = 0.0
        for name, full in self.gripper.items():
            out[name] = full * dc.gripper_profile(fraction, self.args) if self.args.eight_gripper else 0.0
        for name, (amp, centre, cycles, lo, hi) in self.head.items():
            out[name] = min(hi, max(lo, centre + amp * math.sin(2 * math.pi * cycles * fraction)))
        return out


class Wave(Act):
    """One arm raised forward, forearm folded up, waving with elbow yaw."""

    def __init__(self, args, side, base, gripper, head, head_hold=False):
        self.name = f"wave-{side}"
        self.args = args
        self.side = side
        self.base = base
        self.gripper = gripper
        # With head_hold the head is somebody else's (the face overlay keeps
        # it on the person being greeted), so the nod is left out.
        self.head = {} if head_hold else head
        self.ramp = args.wave_ramp
        self.duration = 2 * args.wave_ramp + args.wave_hold

    def blend(self, t):
        """0 at rest, 1 in the raised pose, smooth both ways."""
        if t < self.ramp:
            return dc.smoothstep(t / self.ramp)
        if t > self.duration - self.ramp:
            return 1.0 - dc.smoothstep((t - (self.duration - self.ramp)) / self.ramp)
        return 1.0

    def targets(self, t):
        args = self.args
        s = self.blend(t)
        out = dict(self.base)
        arm = dc.ARMS[self.side]
        pose = {"sh_p": args.wave_sh_p, "sh_r": args.wave_sh_r, "elb_p": args.wave_elb_p}
        for key, raised in pose.items():
            name = arm[key]
            out[name] = self.base[name] + s * (raised - self.base[name])
        # The wave itself: elbow yaw about zero, scaled by the blend so it
        # starts and ends at rest with the ramp.
        tw = t - self.ramp
        if arm["elb_y"] in out:
            out[arm["elb_y"]] = s * args.wave_amp * math.sin(2 * math.pi * args.wave_hz * tw)
        # One gripper fold in the middle of the hold, ramps inside the 0.5 rad/s
        # gripper ceiling.
        name = arm["gripper"]
        if name in self.gripper:
            full = self.gripper[name]
            hold = args.wave_hold
            fold_ramp = args.wave_gripper_ramp
            fold_hold = max(0.0, hold - 2 * fold_ramp) * 0.5
            start = self.ramp + (hold - (2 * fold_ramp + fold_hold)) / 2
            if fold_ramp > 0 and start >= self.ramp:
                out[name] = full * self._fold(t - start, fold_ramp, fold_hold)
        # The head nods with the wave; yaw stays at base (its sign toward the
        # waving arm is not verified).
        if "head_p" in self.head:
            amp, centre, _, lo, hi = self.head["head_p"]
            out["head_p"] = min(hi, max(lo, centre + s * amp * math.sin(2 * math.pi * args.wave_hz * 0.5 * tw)))
        return out

    @staticmethod
    def _fold(u, ramp, hold):
        if u < 0:
            return 0.0
        if u < ramp:
            return dc.smoothstep(u / ramp)
        if u < ramp + hold:
            return 1.0
        if u < 2 * ramp + hold:
            return 1.0 - dc.smoothstep((u - ramp - hold) / ramp)
        return 0.0


class Rest(Act):
    def __init__(self, args, base, head):
        self.name = "rest"
        self.duration = args.rest_seconds
        self.base = base
        self.head = head

    def targets(self, t):
        out = dict(self.base)
        if "head_p" in self.head and self.duration > 0:
            amp, centre, _, lo, hi = self.head["head_p"]
            # A small nod once over the rest, back where it started.
            out["head_p"] = min(hi, max(lo, centre - 0.5 * amp * math.sin(math.pi * t / self.duration)))
        return out


# ---------------------------------------------------------------- setup

def load_config(path):
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    joints = {j["name"]: j for j in cfg["joints"]}
    base = dict(cfg.get("poses", {}).get("base", {}))
    for name in joints:
        base.setdefault(name, 0.0)
    return cfg, joints, base


def limits_from_config(joints):
    return {n: {"min_rad": j["limits"]["min_rad"], "max_rad": j["limits"]["max_rad"]}
            for n, j in joints.items()}


def gripper_reach(limits, amount):
    """Per gripper, how far it folds: toward the far end of its learned range,
    never closer than 10 % to the limit (demo_circle's rule)."""
    out = {}
    for side in ("l", "r"):
        name = dc.ARMS[side]["gripper"]
        if name in limits and amount > 0:
            lo, hi = limits[name]["min_rad"], limits[name]["max_rad"]
            sign = 1.0 if abs(hi) >= abs(lo) else -1.0
            out[name] = sign * min(amount, (abs(hi) if sign > 0 else abs(lo)) * 0.9)
    return out


def head_motion(limits, base, yaw, pitch):
    out = {}
    for name, amp, centre, cycles in (("head_y", yaw, base.get("head_y", 0.0), 1.0),
                                      ("head_p", pitch, base.get("head_p", 0.0), 2.0)):
        if name in limits and amp > 0:
            lo, hi = limits[name]["min_rad"], limits[name]["max_rad"]
            lo, hi = lo + 0.05 * (hi - lo), hi - 0.05 * (hi - lo)
            out[name] = (amp, centre, cycles, lo, hi)
    return out


def parse_playlist(text):
    items = []
    for raw in text.split(","):
        raw = raw.strip()
        if not raw:
            continue
        name, _, count = raw.partition("*")
        name = name.strip().lower()
        if name not in ("eight", "wave", "rest"):
            raise SystemExit(f"unknown act {name!r}; the acts are eight, wave, rest")
        repeats = int(count) if count else 1
        if repeats < 1:
            raise SystemExit(f"{raw}: the repeat count must be at least 1")
        items.append((name, repeats))
    if not items:
        raise SystemExit("the playlist is empty")
    return items


class Cast:
    """Builds acts from the playlist, alternating the waving arm."""

    def __init__(self, args, base, limits):
        self.args = args
        self.base = base
        self.gripper = gripper_reach(limits, args.gripper)
        self.head = head_motion(limits, base, args.head_yaw, args.head_pitch)
        self.waves = 0
        self.eight = None

    def make(self, name, repeats):
        if name == "eight":
            if self.eight is None or self.eight.repeats != repeats:
                self.eight = Eight(self.args, repeats, self.base, self.gripper, self.head)
            return self.eight
        if name == "wave":
            sides = self.args.wave_arms
            side = sides[self.waves % len(sides)]
            self.waves += 1
            return Wave(self.args, side, self.base, self.gripper, self.head)
        return Rest(self.args, self.base, self.head)


def check_act(act, limits):
    """Ranges per joint, the worst step between consecutive setpoints, and
    any joint the act would push outside its envelope."""
    samples = act.sample(int(max(20, act.duration * 20)))
    ranges = {}
    step = {}
    for i, s in enumerate(samples):
        for name, val in s.items():
            lo, hi = ranges.get(name, (val, val))
            ranges[name] = (min(lo, val), max(hi, val))
            if i:
                step[name] = max(step.get(name, 0.0), abs(val - samples[i - 1][name]))
    over = {n: r for n, r in ranges.items()
            if n in limits and (r[0] < limits[n]["min_rad"] or r[1] > limits[n]["max_rad"])}
    return ranges, step, over


def describe(act, limits):
    ranges, step, over = check_act(act, limits)
    print(f"  {act.name}: {act.duration:.1f} s")
    for name, (lo, hi) in ranges.items():
        if name not in limits:
            continue
        lim = limits[name]
        moving = hi - lo > 1e-6
        if not moving and abs(lo) < 1e-6:
            continue
        margin = min(lo - lim["min_rad"], lim["max_rad"] - hi)
        flag = "  <-- OUTSIDE" if name in over else ""
        print(f"    {name:10s} {lo:+.3f} .. {hi:+.3f} rad   step {step.get(name, 0.0):.3f}   "
              f"margin {margin:+.3f}{flag}")
    return over


# ---------------------------------------------------------------- the panel process

class Supervisor:
    """Owns the panel process when --launch is given; otherwise only knows
    how to ask an existing panel to reset."""

    def __init__(self, args):
        self.args = args
        self.proc = None
        self.log_path = None
        self.panel = Client(args.panel)

    @property
    def owns_panel(self):
        return self.args.launch is not None

    def launch(self):
        args = self.args
        exe = args.panel_exe or os.path.join(ROOT, "target", "release", "drlabs.exe")
        addr = args.panel.split("//", 1)[-1].rstrip("/")
        os.makedirs(args.log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.log_path = os.path.join(args.log_dir, f"panel-{stamp}.log")
        env = dict(os.environ, DRLABS_DXL_TRACE="1")
        flags = 0
        if os.name == "nt":
            # Its own group, so the console's Ctrl+C reaches the show and not
            # the panel: the show stops the machine first, then the panel.
            flags = subprocess.CREATE_NEW_PROCESS_GROUP
        log(f"starting panel: {exe} panel -addr {addr} {args.launch}  (log {self.log_path})")
        out = open(self.log_path, "ab")
        self.proc = subprocess.Popen([exe, "panel", "-addr", addr, args.launch],
                                     stdout=out, stderr=subprocess.STDOUT, env=env,
                                     cwd=ROOT, creationflags=flags)
        self.wait_ready(args.bringup_timeout)

    def wait_ready(self, timeout):
        """Until every joint is armed, or the panel has given up."""
        deadline = time.time() + timeout
        last = ""
        while time.time() < deadline:
            if self.proc is not None and self.proc.poll() is not None:
                raise Interrupted(f"the panel exited with code {self.proc.returncode} (see {self.log_path})")
            try:
                state = self.panel.get("/api/state")
            except Interrupted:
                time.sleep(0.5)
                continue
            joints = state["joints"]
            armed = sum(1 for j in joints if j["armed"] and not j["estop"])
            if state.get("connected") and armed == len(joints):
                log(f"panel ready: {armed} joints armed")
                return
            stopped = stopped_joints({j["name"]: j for j in joints})
            blocked = {j["name"]: j["blocked"] for j in joints if j["blocked"]}
            line = f"waiting: {armed}/{len(joints)} armed"
            if stopped:
                line += "; e-stop " + "; ".join(stopped[:3])
            if blocked:
                first = next(iter(blocked.items()))
                line += f"; blocked {first[0]}: {' '.join(first[1])}" + (f" (+{len(blocked)-1})" if len(blocked) > 1 else "")
            if line != last:
                log(line)
                last = line
            time.sleep(0.5)
        raise Interrupted(f"the panel did not arm every joint within {timeout:.0f} s")

    def stop_panel(self, release=True):
        """Torque off through the panel, then end the process.

        The release is an e-stop, not /api/stop: the panel brings its joints
        up with auto-arm, so a plain stop is re-armed on the next cycle and
        the torque never goes off (measured: 15 joints armed 2 s after
        /api/stop, torque still 1 on the servos). An e-stop latches, and its
        stop burst carries the torque-off write (measured: torque 0 on every
        servo after the panel was ended). The panel process has no Ctrl+C
        handler of its own, so ending it any other way leaves the machine
        energised."""
        if release:
            try:
                self.panel.post("/api/estop", {})
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    joints = self.panel.get("/api/state")["joints"]
                    if not any(j["armed"] for j in joints):
                        break
                    time.sleep(0.2)
                time.sleep(1.0)  # a few control cycles, so the torque-off phase goes out
            except Interrupted:
                pass
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def reset_existing(self):
        r = self.panel.post("/api/reset", {})
        if "error" in r:
            raise Interrupted(f"reset refused: {r['error']}")
        self.wait_ready(self.args.reset_timeout)

    def recover(self, reason):
        """Keeps trying until the machine is armed again. Returns the number
        of attempts it took."""
        note("estop", reason)
        attempt = 0
        while True:
            attempt += 1
            delay = min(60.0, 5.0 * 2 ** (attempt - 1))
            note("recovery", f"attempt {attempt}: waiting {delay:.0f} s, then "
                             + ("restarting the panel" if self.owns_panel else "POST /api/reset"))
            time.sleep(delay)
            try:
                if self.owns_panel:
                    self.stop_panel(release=False)
                    self.launch()
                else:
                    self.reset_existing()
                note("recovery", f"recovered after {attempt} attempt(s)")
                return attempt
            except Interrupted as error:
                note("recovery", f"attempt {attempt} failed: {error}")
                if self.owns_panel:
                    self.stop_panel(release=False)


# ---------------------------------------------------------------- the face

class HeadOverlay:
    """The head, when a camera is on it: follows the face tracker's candidate
    or sweeps the room. Called on every setpoint; keeps its own idea of where
    the head is, since the tracker's step is relative.

    Joint signs come from --head-yaw-sign / --head-pitch-sign, which
    `calibrate_head` measures; with the wrong sign the head runs away from
    the face and pins at a limit, which is obvious within a second.
    """

    def __init__(self, tracker, args, base, limits):
        self.tracker = tracker
        self.args = args
        # Which joint pans and which tilts is measured, not assumed: on this
        # robot the joint called head_p pans and head_y tilts (2026-09-05,
        # --calibrate-head), whatever the names say.
        self.yaw_joint = args.head_yaw_joint
        self.pitch_joint = args.head_pitch_joint
        self.head_joints = (self.yaw_joint, self.pitch_joint)
        self.base_yaw = base.get(self.yaw_joint, 0.0)
        self.base_pitch = base.get(self.pitch_joint, 0.0)
        self.yaw_range = self._inset(limits[self.yaw_joint])
        self.pitch_range = self._inset(limits[self.pitch_joint])
        self.yaw = self.base_yaw
        self.pitch = self.base_pitch
        self.candidate = None

    @staticmethod
    def _inset(lim):
        lo, hi = lim["min_rad"], lim["max_rad"]
        return (lo + 0.05 * (hi - lo), hi - 0.05 * (hi - lo))

    def resync(self, panel):
        """Adopt the head's actual position (after a restart the servos have
        been re-armed wherever they were)."""
        try:
            joints = panel.joints()
            if joints[self.yaw_joint]["position_rad"] is not None:
                self.yaw = joints[self.yaw_joint]["position_rad"]
            if joints[self.pitch_joint]["position_rad"] is not None:
                self.pitch = joints[self.pitch_joint]["position_rad"]
        except (Interrupted, KeyError):
            pass

    def __call__(self, targets):
        args = self.args
        self.yaw, self.pitch, self.candidate = self.tracker.head_step(
            self.yaw, self.pitch, args.head_yaw_sign, args.head_pitch_sign,
            self.base_yaw, self.base_pitch, self.yaw_range, self.pitch_range,
            gain=args.head_gain, search_amplitude=args.search_amplitude,
            search_period=args.search_period)
        out = dict(targets)
        out[self.yaw_joint] = self.yaw
        out[self.pitch_joint] = self.pitch
        return out

    def side_of_person(self):
        """Which arm is nearer the person the head is turned to. With the
        calibrated sign, a positive turn is toward the robot's right."""
        turn = (self.yaw - self.base_yaw) * self.args.head_yaw_sign
        if abs(turn) < 0.1:
            # Straight ahead: take turns.
            self.alternate = "l" if getattr(self, "alternate", "l") == "r" else "r"
            return self.alternate
        return "r" if turn > 0 else "l"

    def summary(self):
        snap = self.tracker.snapshot()
        return {"faces": len(snap["tracks"]), "fps": snap["fps"], "error": snap["error"],
                "candidate": self.candidate, "tracks": snap["tracks"],
                "head": {"yaw": round(self.yaw, 3), "pitch": round(self.pitch, 3)}}


def calibrate_head(panel, tracker, args, base):
    """Measures the head joints' signs from the picture: turn the head a
    little, see which way the scene slides.

    A camera panning right slides the scene left. `head_step` adds
    yaw_sign * bearing, where a face on the image's right has a positive
    bearing and needs the camera to pan right — so yaw_sign is +1 when a
    positive head_y slides the scene left (dx < 0). Likewise a camera
    tilting up slides the scene down; a face above centre has a negative
    bearing (image y is down) and needs a tilt up — so pitch_sign is -1 when
    a positive head_p slides the scene down (dy > 0).
    """
    import cv2
    import numpy as np

    def gray():
        deadline = time.time() + 5
        while time.time() < deadline:
            with tracker.lock:
                frame = None if tracker.frame is None else tracker.frame.copy()
            if frame is not None:
                return np.float32(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            time.sleep(0.1)
        raise SystemExit("no camera frame for the calibration")

    def shift(joint, delta, rest):
        approach(panel, {joint: rest}, args, f"[calibrate {joint} rest]")
        time.sleep(0.5)
        before = gray()
        approach(panel, {joint: rest + delta}, args, f"[calibrate {joint} +{delta}]")
        time.sleep(0.5)
        after = gray()
        (dx, dy), response = cv2.phaseCorrelate(before, after)
        approach(panel, {joint: rest}, args, f"[calibrate {joint} back]")
        return dx, dy, response

    # Each joint is turned in turn; the one that slides the scene sideways is
    # the pan joint, whatever it is called. (Measured here 2026-09-05: the
    # joint named head_p pans and head_y tilts.)
    results = {}
    for joint in ("head_y", "head_p"):
        dx, dy, resp = shift(joint, args.calibrate_delta, base.get(joint, 0.0))
        results[joint] = (dx, dy, resp)
        log(f"{joint} +{args.calibrate_delta} rad: scene shift dx={dx:+.1f} dy={dy:+.1f} px (response {resp:.2f})")
    weak = [j for j, (_, _, r) in results.items() if r < 0.05]
    if weak:
        log(f"WARNING: weak phase-correlation response on {weak}; point the camera at a "
            f"textured, static scene and run again")
    yaw_joint = max(results, key=lambda j: abs(results[j][0]) - abs(results[j][1]))
    pitch_joint = "head_p" if yaw_joint == "head_y" else "head_y"
    dx = results[yaw_joint][0]
    dy = results[pitch_joint][1]
    yaw_sign = 1.0 if dx < 0 else -1.0
    pitch_sign = -1.0 if dy > 0 else 1.0
    log(f"{yaw_joint} pans (dx {dx:+.1f} px), {pitch_joint} tilts (dy {dy:+.1f} px)")
    print(f"\n    --head-yaw-joint {yaw_joint} --head-yaw-sign {yaw_sign:+.0f} "
          f"--head-pitch-joint {pitch_joint} --head-pitch-sign {pitch_sign:+.0f}\n")
    return yaw_joint, yaw_sign, pitch_joint, pitch_sign


# ---------------------------------------------------------------- the runner

def approach(panel, targets, args, label, overlay=None):
    """A profiled move to `targets`, and a wait until it has arrived.

    Arrival is: no trajectory still running (`moving_to_rad` is null once the
    profile is spent) and every joint within --settle-tol of its target.
    demo_circle's wait_settled tests position error alone, but target_rad
    follows the profile while it runs, so the error is small from the first
    cycle and the wait returned before the arm had moved (measured: 0.76 rad
    behind at the first setpoint of the act).
    """
    if overlay is not None:
        # The head is the overlay's: it is left out of the profiled move and
        # keeps following (or searching) while the arms travel.
        targets = {n: v for n, v in targets.items() if n not in overlay.head_joints}
    r = panel.move(targets, args.approach_velocity, args.approach_velocity * 2)
    if "error" in r:
        raise Interrupted(f"{label}: move refused: {r['error']}")
    names = list(targets)
    deadline = time.time() + args.settle_timeout
    worst = ("?", 0.0)
    while time.time() < deadline:
        if overlay is not None:
            r = panel.pose(overlay({}))
            if "error" in r:
                raise Interrupted(f"{label}: head pose refused: {r['error']}")
        joints = panel.joints()
        stopped = stopped_joints(joints, names)
        if stopped:
            raise Interrupted(f"{label}: e-stop on " + ", ".join(stopped))
        moving = [n for n in names if joints[n]["moving_to_rad"] is not None]
        errors = {n: abs(joints[n]["position_rad"] - joints[n]["target_rad"])
                  for n in names if joints[n]["position_rad"] is not None}
        worst = max(errors.items(), key=lambda kv: kv[1]) if errors else worst
        if not moving and all(e < args.settle_tol for e in errors.values()):
            return
        time.sleep(0.1)
    raise Interrupted(f"{label}: joints did not settle in {args.settle_timeout:.0f} s "
                      f"(worst {worst[0]} off by {worst[1]:.3f} rad, tolerance {args.settle_tol})")


def stream(panel, act, args, label, overlay=None, cut=None, status=None):
    """Streams the act's setpoints at --rate until its duration is up.

    `overlay(targets)` may rewrite a setpoint before it goes out (the head
    following a face); `cut()` is asked every tick and may raise Cut to end
    the act early on purpose; `status` is extra fields for the monitor page.
    """
    dt = 1.0 / args.rate
    steps = int(act.duration * args.rate)
    names = list(act.targets(0.0))
    arm_names = [n for n in names if any(k in n for k in ("sh_", "elb_", "wrist"))]
    t0 = time.perf_counter()
    for i in range(steps + 1):
        t = min(act.duration, i * dt)
        targets = act.targets(t)
        if overlay is not None:
            targets = overlay(targets)
        r = panel.pose(targets)
        if "error" in r:
            joints = panel.joints()
            stopped = stopped_joints(joints, names)
            raise Interrupted(f"{label} t={t:.1f}s: pose refused: {r['error']}"
                              + ("; e-stop on " + ", ".join(stopped) if stopped else ""))
        if cut is not None:
            cut()
        if i % int(args.rate) == 0:
            joints = panel.joints()
            stopped = stopped_joints(joints, names)
            if stopped:
                raise Interrupted(f"{label} t={t:.1f}s: e-stop on " + ", ".join(stopped))
            err = max((abs(joints[n]["position_rad"] - joints[n]["target_rad"])
                       for n in arm_names if joints[n]["position_rad"] is not None), default=0.0)
            volts = [joints[n]["voltage_v"] for n in arm_names if joints[n]["voltage_v"] and joints[n]["voltage_v"] > 8]
            log(f"{label} t={t:5.1f}s  arm err {err:.3f} rad  V min {min(volts):.1f}" if volts
                else f"{label} t={t:5.1f}s  arm err {err:.3f} rad")
            if MONITOR is not None:
                MONITOR.set_status(t=round(t, 1), **(status or {}))
        next_at = t0 + (i + 1) * dt
        sleep = next_at - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)


class Quit(Exception):
    """The monitor page asked the show to finish."""


def run_show(args, cast, playlist, base, limits, supervisor, faces=None):
    """The playlist, forever. `faces` is the HeadOverlay in --face mode: it
    rewrites the head in every setpoint and turns a 3 s look into a greeting
    that cuts into the current act."""
    panel = supervisor.panel
    cycle = 0
    events = 0
    greetings = 0

    def cut():
        if MONITOR is not None and MONITOR.quit_requested:
            raise Cut("quit")
        if faces is not None:
            ready = faces.tracker.greet_ready()
            if ready is not None:
                raise Cut("greet", ready)

    def status(act, **more):
        if MONITOR is not None:
            fields = {"cycle": cycle, "act": act.name, "recoveries": events,
                      "greetings": greetings, "recovering": False}
            if faces is not None:
                fields["tracker"] = faces.summary()
            fields.update(more)
            MONITOR.set_status(**fields)

    def back_to_base(label):
        while True:
            try:
                approach(panel, base, args, label, overlay=faces)
                return
            except Interrupted as again:
                recover(str(again))

    def recover(reason):
        nonlocal events
        events += 1
        if args.max_recoveries and events > args.max_recoveries:
            raise SystemExit(f"giving up after {events - 1} recoveries: {reason}")
        if MONITOR is not None:
            MONITOR.set_status(recovering=True, recoveries=events)
        supervisor.recover(reason)
        if faces is not None:
            faces.resync(panel)

    def greet(track):
        """Head stays on the person (the overlay keeps it there); the nearer
        arm waves. Then that person is done with, and the show resumes."""
        nonlocal greetings
        side = faces.side_of_person()
        greetings += 1
        note("greet", f"#{greetings}: face #{track['id']} at {track['distance_m'] or 0:.2f} m looked for "
                      f"{track['looking_for']:.1f} s; waving with the {'left' if side == 'l' else 'right'} arm")
        wave = Wave(args, side, base, cast.gripper, cast.head, head_hold=True)
        label = f"[cycle {cycle} greet {wave.name}]"
        approach(panel, wave.targets(0.0), args, label, overlay=faces)
        status(wave, greeting=track["id"])
        stream(panel, wave, args, label, overlay=faces, status={"greeting": track["id"]})
        faces.tracker.mark_greeted(track["id"])
        note("greet", f"done; face #{track['id']} is greeted, searching for someone else")

    while True:
        cycle += 1
        for name, repeats in playlist:
            act = cast.make(name, repeats)
            label = f"[cycle {cycle} {act.name}]"
            note("act", f"{label} starting ({act.duration:.0f} s)")
            while True:
                try:
                    status(act)
                    approach(panel, act.targets(0.0), args, label, overlay=faces)
                    stream(panel, act, args, label, overlay=faces, cut=cut)
                    break
                except Cut as cut_reason:
                    if cut_reason.reason == "quit":
                        raise Quit()
                    try:
                        back_to_base(f"[cycle {cycle} greet approach]")
                        greet(cut_reason.track)
                    except Interrupted as error:
                        recover(str(error))
                        back_to_base("[base]")
                    log(f"{label} restarting after the greeting")
                except Interrupted as error:
                    recover(str(error))
                    # Back to base under a profile before the act starts over:
                    # after a servo reboot the arms hang wherever gravity left them.
                    back_to_base("[base]")
                    log(f"{label} restarting after recovery #{events}")
        if args.once:
            log("one cycle done; returning to base")
            approach(panel, base, args, "[base]", overlay=faces)
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--panel", default="http://127.0.0.1:8085")
    ap.add_argument("--launch", metavar="ROBOT.json", default=None,
                    help="start the panel on this description and restart it to recover")
    ap.add_argument("--config", default=None,
                    help="the description, for limits and the base pose (default: --launch, else configs/ubp15.json)")
    ap.add_argument("--panel-exe", default=None, help="drlabs.exe (default target/release/drlabs.exe)")
    ap.add_argument("--log-dir", default=os.path.join(tempfile.gettempdir(), "drlabs-show"))
    ap.add_argument("--playlist", default="eight*3,wave,rest")
    ap.add_argument("--once", action="store_true", help="one pass of the playlist, then base")
    ap.add_argument("--rate", type=float, default=20.0, help="setpoints per second")
    ap.add_argument("--upper", type=float, default=0.25)
    ap.add_argument("--lower", type=float, default=0.25)
    ap.add_argument("--sh-p-sign", type=float, default=-1.0, choices=[1.0, -1.0])
    # eight
    ap.add_argument("--period", type=float, default=12.0, help="seconds per figure-eight")
    ap.add_argument("--eight-cx", type=float, default=0.28, help="centre, forward of the torso, m")
    ap.add_argument("--eight-cz", type=float, default=0.28, help="centre height, torso z, m (shoulder 0.60)")
    ap.add_argument("--eight-ax", type=float, default=0.05, help="half width, m")
    ap.add_argument("--eight-az", type=float, default=0.08, help="half height, m")
    ap.add_argument("--eight-gripper", action="store_true", help="fold the grippers once per eight")
    ap.add_argument("--gripper-at", type=float, default=0.35)
    ap.add_argument("--gripper-ramp", type=float, default=0.20)
    ap.add_argument("--gripper-hold", type=float, default=0.10)
    # wave
    ap.add_argument("--wave-arms", default="rl", help="which arm waves, in turn: r, l or rl")
    ap.add_argument("--wave-sh-p", type=float, default=-0.75, help="shoulder pitch of the raised arm, rad (negative = forward)")
    ap.add_argument("--wave-sh-r", type=float, default=-0.35, help="shoulder roll, rad (negative = outward)")
    ap.add_argument("--wave-elb-p", type=float, default=-1.4, help="elbow, rad (negative = flexed)")
    ap.add_argument("--wave-amp", type=float, default=0.4, help="elbow yaw swing, rad")
    ap.add_argument("--wave-hz", type=float, default=0.5, help="waves per second")
    ap.add_argument("--wave-ramp", type=float, default=3.0, help="seconds to raise and to lower the arm")
    ap.add_argument("--wave-hold", type=float, default=6.0, help="seconds spent waving")
    ap.add_argument("--wave-gripper-ramp", type=float, default=2.0)
    # shared
    ap.add_argument("--gripper", type=float, default=0.8, help="how far a gripper folds, rad; 0 disables")
    ap.add_argument("--head-yaw", type=float, default=0.25)
    ap.add_argument("--head-pitch", type=float, default=0.20)
    ap.add_argument("--rest-seconds", type=float, default=2.0)
    ap.add_argument("--approach-velocity", type=float, default=0.3, help="rad/s ceiling for profiled moves between acts")
    ap.add_argument("--settle-tol", type=float, default=0.12)
    ap.add_argument("--settle-timeout", type=float, default=30.0)
    ap.add_argument("--bringup-timeout", type=float, default=90.0)
    ap.add_argument("--reset-timeout", type=float, default=15.0)
    ap.add_argument("--max-recoveries", type=int, default=0, help="0 = never give up")
    ap.add_argument("--dry-run", action="store_true")
    # face
    ap.add_argument("--face", action="store_true",
                    help="track faces with the head camera and greet whoever looks for --look-seconds "
                         "(needs pyrealsense2, opencv, mediapipe: run under imitation_upper's .venv)")
    ap.add_argument("--face-model", default=None, help="face_landmarker.task (default tools/models/)")
    ap.add_argument("--head-yaw-joint", default="head_p",
                    help="the joint that pans the camera (measured with --calibrate-head; on this "
                         "robot it is the one named head_p)")
    ap.add_argument("--head-pitch-joint", default="head_y", help="the joint that tilts the camera")
    ap.add_argument("--head-yaw-sign", type=float, default=-1.0, choices=[1.0, -1.0],
                    help="+1 when a positive pan joint turns the camera toward the image's right (see --calibrate-head)")
    ap.add_argument("--head-pitch-sign", type=float, default=-1.0, choices=[1.0, -1.0],
                    help="+1 when a positive tilt joint turns the camera toward the image's bottom")
    ap.add_argument("--head-gain", type=float, default=0.6, help="fraction of the face's bearing applied per tick")
    ap.add_argument("--search-amplitude", type=float, default=0.5, help="rad either side of base while searching")
    ap.add_argument("--search-period", type=float, default=8.0, help="seconds per search sweep")
    ap.add_argument("--look-seconds", type=float, default=3.0, help="how long a face must look before a greeting")
    ap.add_argument("--look-yaw-deg", type=float, default=20.0)
    ap.add_argument("--look-pitch-deg", type=float, default=20.0)
    ap.add_argument("--regreet-after", type=float, default=10.0,
                    help="seconds out of view after which a greeted person counts as somebody new")
    ap.add_argument("--calibrate-head", action="store_true",
                    help="measure the head joint signs from the camera picture, print them, exit")
    ap.add_argument("--calibrate-delta", type=float, default=0.15, help="rad to turn for the calibration")
    # monitor
    ap.add_argument("--monitor-addr", default="127.0.0.1:8086", help="the monitoring page; 'off' disables it")
    args = ap.parse_args()

    if any(s not in "lr" for s in args.wave_arms) or not args.wave_arms:
        raise SystemExit("--wave-arms must be r, l or rl")
    config_path = args.config or args.launch or os.path.join(ROOT, "configs", "ubp15.json")
    _, cfg_joints, base = load_config(config_path)
    limits = limits_from_config(cfg_joints)
    playlist = parse_playlist(args.playlist)
    cast = Cast(args, base, limits)

    # Every act's path against the description, before anything moves.
    print(f"playlist: " + ", ".join(f"{n}*{r}" if r > 1 else n for n, r in playlist)
          + f"  (waving arms: {args.wave_arms}, {'once' if args.once else 'forever'})")
    preview = Cast(args, base, limits)
    bad = {}
    seen = set()
    for name, repeats in playlist:
        acts = [preview.make(name, repeats)]
        if name == "wave":
            acts += [preview.make(name, repeats) for _ in range(len(args.wave_arms) - 1)]
        for act in acts:
            if act.name in seen:
                continue
            seen.add(act.name)
            over = describe(act, limits)
            bad.update({f"{act.name}:{k}": v for k, v in over.items()})
    if bad:
        for key, (lo, hi) in bad.items():
            print(f"  {key} reaches {lo:+.3f} .. {hi:+.3f} rad, outside the envelope")
        raise SystemExit("an act leaves a joint's envelope; adjust its parameters")
    if args.dry_run:
        return

    global MONITOR
    if args.monitor_addr.lower() != "off":
        import show_monitor
        host, _, port = args.monitor_addr.rpartition(":")
        MONITOR = show_monitor.Monitor(args.panel, (host or "127.0.0.1", int(port)))
        log(f"monitor page on {MONITOR.start()}")

    tracker = None
    faces = None
    if args.face or args.calibrate_head:
        try:
            import face_tracker
        except ImportError as error:
            raise SystemExit(f"--face needs pyrealsense2, opencv and mediapipe ({error}); run under "
                             f"C:\\pj_depthcam_teleop\\imitation_upper\\.venv\\Scripts\\python.exe")
        tracker = face_tracker.FaceTracker(
            model=args.face_model or face_tracker.DEFAULT_MODEL,
            look_seconds=args.look_seconds, look_yaw_deg=args.look_yaw_deg,
            look_pitch_deg=args.look_pitch_deg, regreet_after=args.regreet_after)
        tracker.start()
        log("face tracker started")
        if MONITOR is not None:
            MONITOR.camera = tracker.jpeg
        if args.face:
            faces = HeadOverlay(tracker, args, base, limits)

    supervisor = Supervisor(args)

    def finish(why):
        """base pose, release, panel down — or just hold if the panel is not ours."""
        if supervisor.owns_panel:
            note("quit", f"{why}; returning to base, then releasing the machine")
            try:
                approach(supervisor.panel, base, args, "[base]")
            except (Interrupted, KeyboardInterrupt) as error:
                log(f"base return skipped: {error}")
            supervisor.stop_panel(release=True)
            log("panel stopped")
        else:
            note("quit", f"{why}; holding the last setpoint "
                         "(the panel's base button, or POST /api/move {\"pose\":\"base\"}, returns it)")

    try:
        if supervisor.owns_panel:
            try:
                supervisor.launch()
            except Interrupted as error:
                supervisor.stop_panel(release=False)
                raise SystemExit(f"the panel did not come up: {error}")
        else:
            try:
                supervisor.wait_ready(5.0)
            except Interrupted as error:
                raise SystemExit(f"no armed panel at {args.panel}: {error}")
        if faces is not None:
            faces.resync(supervisor.panel)
        if args.calibrate_head:
            calibrate_head(supervisor.panel, tracker, args, base)
            finish("calibration done")
            return
        run_show(args, cast, playlist, base, limits, supervisor, faces)
    except KeyboardInterrupt:
        print()
        finish("stopped by user")
        return
    except Quit:
        finish("quit from the monitor page")
        return
    finally:
        if tracker is not None:
            tracker.stop()
    if supervisor.owns_panel:
        supervisor.stop_panel(release=True)


if __name__ == "__main__":
    main()
