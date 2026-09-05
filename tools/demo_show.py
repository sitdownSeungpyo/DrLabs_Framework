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

    def __init__(self, args, side, base, gripper, head):
        self.name = f"wave-{side}"
        self.args = args
        self.side = side
        self.base = base
        self.gripper = gripper
        self.head = head
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
        log(f"RECOVERY: {reason}")
        attempt = 0
        while True:
            attempt += 1
            delay = min(60.0, 5.0 * 2 ** (attempt - 1))
            log(f"recovery attempt {attempt}: waiting {delay:.0f} s")
            time.sleep(delay)
            try:
                if self.owns_panel:
                    self.stop_panel(release=False)
                    self.launch()
                else:
                    self.reset_existing()
                log(f"recovered after {attempt} attempt(s)")
                return attempt
            except Interrupted as error:
                log(f"recovery attempt {attempt} failed: {error}")
                if self.owns_panel:
                    self.stop_panel(release=False)


# ---------------------------------------------------------------- the runner

def approach(panel, targets, args, label):
    """A profiled move to `targets`, and a wait until it has arrived.

    Arrival is: no trajectory still running (`moving_to_rad` is null once the
    profile is spent) and every joint within --settle-tol of its target.
    demo_circle's wait_settled tests position error alone, but target_rad
    follows the profile while it runs, so the error is small from the first
    cycle and the wait returned before the arm had moved (measured: 0.76 rad
    behind at the first setpoint of the act).
    """
    r = panel.move(targets, args.approach_velocity, args.approach_velocity * 2)
    if "error" in r:
        raise Interrupted(f"{label}: move refused: {r['error']}")
    names = list(targets)
    deadline = time.time() + args.settle_timeout
    worst = ("?", 0.0)
    while time.time() < deadline:
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


def stream(panel, act, args, label):
    """Streams the act's setpoints at --rate until its duration is up."""
    dt = 1.0 / args.rate
    steps = int(act.duration * args.rate)
    names = list(act.targets(0.0))
    arm_names = [n for n in names if any(k in n for k in ("sh_", "elb_", "wrist"))]
    t0 = time.perf_counter()
    for i in range(steps + 1):
        t = min(act.duration, i * dt)
        r = panel.pose(act.targets(t))
        if "error" in r:
            joints = panel.joints()
            stopped = stopped_joints(joints, names)
            raise Interrupted(f"{label} t={t:.1f}s: pose refused: {r['error']}"
                              + ("; e-stop on " + ", ".join(stopped) if stopped else ""))
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
        next_at = t0 + (i + 1) * dt
        sleep = next_at - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)


def run_show(args, cast, playlist, base, limits, supervisor):
    panel = supervisor.panel
    cycle = 0
    events = 0
    while True:
        cycle += 1
        for name, repeats in playlist:
            act = cast.make(name, repeats)
            label = f"[cycle {cycle} {act.name}]"
            while True:
                try:
                    approach(panel, act.targets(0.0), args, label)
                    stream(panel, act, args, label)
                    break
                except Interrupted as error:
                    events += 1
                    if args.max_recoveries and events > args.max_recoveries:
                        raise SystemExit(f"giving up after {events - 1} recoveries: {error}")
                    supervisor.recover(str(error))
                    # Back to base under a profile before the act starts over:
                    # after a servo reboot the arms hang wherever gravity left them.
                    while True:
                        try:
                            approach(panel, base, args, "[base]")
                            break
                        except Interrupted as again:
                            supervisor.recover(str(again))
                    log(f"{label} restarting after recovery #{events}")
        if args.once:
            log("one cycle done; returning to base")
            approach(panel, base, args, "[base]")
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

    supervisor = Supervisor(args)
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
        run_show(args, cast, playlist, base, limits, supervisor)
    except KeyboardInterrupt:
        print()
        if supervisor.owns_panel:
            log("stopped by user; returning to base, then releasing the machine")
            try:
                approach(supervisor.panel, base, args, "[base]")
            except (Interrupted, KeyboardInterrupt) as error:
                log(f"base return skipped: {error}")
            supervisor.stop_panel(release=True)
            log("panel stopped")
        else:
            log("stopped by user; holding the last setpoint "
                "(the panel's base button, or POST /api/move {\"pose\":\"base\"}, returns it)")
        return
    if supervisor.owns_panel:
        supervisor.stop_panel(release=True)


if __name__ == "__main__":
    main()
