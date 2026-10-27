#!/usr/bin/env python3
"""Demo: each arm draws a circle in the air in front of its shoulder.

A throwaway upper-layer program. It talks to a running `drlabs panel` over
its HTTP API, so every setpoint still passes the panel's envelope, e-stop and
velocity limits — this script never touches a bus.

    drlabs panel -addr 127.0.0.1:8085 configs/ubp.json     (in one terminal)
    python tools/demo_circle.py --dry-run                   (check the joint path)
    python tools/demo_circle.py                             (draw)

Kinematics
----------
Per arm, a 3-joint position chain — shoulder pitch, shoulder roll, elbow
pitch — in the shoulder frame: x forward, y outward (away from the body),
z up. At zero every joint hangs the arm straight down along -z. Shoulder
yaw, elbow yaw and wrist pitch are held at zero. The grippers fold once per
revolution and the head looks side to side and nods along with the circle.

With --shared the two hands draw the same circle in unison, side by side —
one in front of each shoulder, in a forward-up plane, at the same phase. A
front-centre shared circle is not possible on this robot: the shoulders
barely adduct (roll only ~+0.08 rad inward), so the hands cannot meet on the
midline. Each arm solves its own IK to its own point (see base_to_shoulder /
shared_offset).

    forward  : raise the hanging arm about the lateral axis, hand comes to +x
    outward  : abduct about the forward axis, hand goes to +y (outward)
    flex     : bend the elbow, forearm folds toward the direction the upper
               arm was raised in

Link lengths come from imitation_upper/config/retarget.yaml (0.25 m / 0.25 m,
marked there as "replace with measured values"); pass --upper/--lower when the
real numbers are known.

Sign conventions in DrLabs joint space (configs/ubp.json), from the ranges
learned by hand and the base pose:
    sh_r  negative = outward      (both arms; l_sh_r range -2.03 … +0.08)
    elb_p negative = flexed       (l_elb_p range -1.62 … +0.66, base -0.785)
    sh_p  negative = forward      (verified on the robot 2026-09-04; --sh-p-sign +1 flips it)
Both arms use the same joint values for the mirror pose; the description's
`direction` field already mirrors the right side at the motor.
"""

import argparse
import json
import math
import sys
import time
import urllib.request
from urllib.error import HTTPError, URLError

ARMS = {
    "l": {"sh_p": "l_sh_p", "sh_r": "l_sh_r", "sh_y": "l_sh_y",
          "elb_p": "l_elb_p", "elb_y": "l_elb_y", "wrist_p": "l_wrist_p",
          "gripper": "l_gripper"},
    "r": {"sh_p": "r_sh_p", "sh_r": "r_sh_r", "sh_y": "r_sh_y",
          "elb_p": "r_elb_p", "elb_y": "r_elb_y", "wrist_p": "r_wrist_p",
          "gripper": "r_gripper"},
}

# Shoulder origins in the torso (base) frame: x forward, y left, z up. From
# models/ubp.urdf, the same geometry as configs/ubp.json. Used only by the
# shared-circle mode, which defines one circle in this frame and reaches both
# hands to it; the mirrored mode works in each shoulder's own frame and needs
# no offsets.
SHOULDER = {"l": (0.0, 0.18, 0.60), "r": (0.0, -0.18, 0.60)}


def smoothstep(t):
    t = min(1.0, max(0.0, t))
    return 0.5 - 0.5 * math.cos(math.pi * t)


def gripper_profile(fraction, args):
    """How far the gripper is folded, 0..1, over one revolution.

    Once per revolution, in the middle of it: fold over `--gripper-ramp` of
    the revolution, hold, unfold over the same, and stay open for the rest.
    """
    a = args.gripper_at
    ramp = args.gripper_ramp
    hold = args.gripper_hold
    if fraction < a:
        return 0.0
    if fraction < a + ramp:
        return smoothstep((fraction - a) / ramp)
    if fraction < a + ramp + hold:
        return 1.0
    if fraction < a + 2 * ramp + hold:
        return 1.0 - smoothstep((fraction - a - ramp - hold) / ramp)
    return 0.0


# ---------------------------------------------------------------- kinematics

def rot_y(a, v):
    """Rotate v about +y by a (right-handed)."""
    x, y, z = v
    return (x * math.cos(a) + z * math.sin(a), y, -x * math.sin(a) + z * math.cos(a))


def rot_x(a, v):
    x, y, z = v
    return (x, y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a))


def fk(q, upper, lower):
    """Wrist position in the shoulder frame for q = (forward, outward, flex).

    Chain: shoulder pitch (about y) → shoulder roll (about x) → elbow (about
    the upper arm's local y). With the arm hanging along -z, a *negative*
    rotation about +y brings the hand forward while a *positive* rotation
    about +x brings it outward (+y); the signs below hide that so that every
    element of q is positive in its named direction.
    """
    forward, outward, flex = q
    hand = (0.0, 0.0, -lower)
    fore = rot_y(-flex, hand)                       # elbow bends toward +x
    wrist_local = (fore[0], fore[1], fore[2] - upper)
    v = rot_x(outward, wrist_local)
    return rot_y(-forward, v)


def elbow_fk(q, upper):
    forward, outward, _ = q
    v = rot_x(outward, (0.0, 0.0, -upper))
    return rot_y(-forward, v)


def solve3(a, b):
    """Solve the 3x3 system a x = b by Gaussian elimination with pivoting."""
    m = [list(a[i]) + [b[i]] for i in range(3)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        if abs(m[c][c]) < 1e-12:
            raise ValueError("singular")
        for r in range(3):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, 4):
                    m[r][k] -= f * m[c][k]
    return [m[i][3] / m[i][i] for i in range(3)]


def ik(target, q0, upper, lower, iters=40, damping=0.02):
    """Damped least squares from q0. Returns (q, residual_m)."""
    q = list(q0)
    for _ in range(iters):
        p = fk(q, upper, lower)
        err = [target[i] - p[i] for i in range(3)]
        res = math.sqrt(sum(e * e for e in err))
        if res < 1e-5:
            break
        # numerical Jacobian, columns = joints
        h = 1e-5
        jac = [[0.0] * 3 for _ in range(3)]
        for j in range(3):
            dq = list(q)
            dq[j] += h
            pj = fk(dq, upper, lower)
            for i in range(3):
                jac[i][j] = (pj[i] - p[i]) / h
        # (JᵀJ + λ²I) dq = Jᵀ err
        jtj = [[sum(jac[k][i] * jac[k][j] for k in range(3)) + (damping ** 2 if i == j else 0.0)
                for j in range(3)] for i in range(3)]
        jte = [sum(jac[k][i] * err[k] for k in range(3)) for i in range(3)]
        try:
            dq = solve3(jtj, jte)
        except ValueError:
            break
        step = math.sqrt(sum(d * d for d in dq))
        if step > 0.3:
            dq = [d * 0.3 / step for d in dq]
        q = [q[i] + dq[i] for i in range(3)]
        # keep the elbow on the natural side
        q[2] = max(0.0, q[2])
    p = fk(q, upper, lower)
    res = math.sqrt(sum((target[i] - p[i]) ** 2 for i in range(3)))
    return q, res


def to_joint_space(q, sh_p_sign):
    forward, outward, flex = q
    return {"sh_p": sh_p_sign * forward, "sh_r": -outward, "elb_p": -flex}


# ---------------------------------------------------------------- panel API

class Panel:
    def __init__(self, base):
        self.base = base.rstrip("/")

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=2) as r:
            return json.loads(r.read().decode())

    def post(self, path, body):
        data = json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=2) as r:
                return json.loads(r.read().decode())
        except HTTPError as error:
            # A refusal (409 on an e-stopped joint, 400 on a bad request)
            # carries its reason in the body; hand it back like any other
            # error reply instead of dying with a traceback.
            try:
                body = json.loads(error.read().decode())
            except (ValueError, OSError):
                body = {}
            reason = body.get("error") if isinstance(body, dict) else None
            return {"error": reason or f"HTTP {error.code} {error.reason}"}

    def joints(self):
        return {j["name"]: j for j in self.get("/api/state")["joints"]}

    def move(self, targets, velocity, accel):
        return self.post("/api/move", {"targets": targets, "velocity": velocity, "accel": accel})

    def move_pose(self, name, velocity, accel):
        return self.post("/api/move", {"pose": name, "velocity": velocity, "accel": accel})

    def pose(self, targets):
        return self.post("/api/pose", targets)


def wait_settled(panel, names, tol, timeout):
    """Wait until every named joint is within `tol` of its target.

    Position error is the whole test — not `progress`. A Dynamixel under
    gravity holds a few degrees short of its goal (measured ~0.08 rad on the
    MX-28 elbows), and the reported progress only tracks a trajectory while
    one is running, so it reads 0 once the profile is spent. Waiting on it
    hangs forever on a machine that has in fact arrived.
    """
    deadline = time.time() + timeout
    last = {}
    while time.time() < deadline:
        joints = panel.joints()
        stopped = [n for n in names if joints[n]["estop"]]
        if stopped:
            raise RuntimeError("e-stop on " + ", ".join(
                f"{n} ({joints[n]['estop_reason']})" for n in stopped))
        last = {n: abs(joints[n]["position_rad"] - joints[n]["target_rad"]) for n in names}
        if all(err < tol for err in last.values()):
            return
        time.sleep(0.1)
    worst = max(last.items(), key=lambda kv: kv[1]) if last else ("?", 0.0)
    raise RuntimeError(f"joints did not settle in {timeout:.0f} s "
                       f"(worst {worst[0]} off by {worst[1]:.3f} rad, tolerance {tol})")


# ---------------------------------------------------------------- the demo

def circle_point(args, phase):
    """A point on the circle, in the shoulder frame: vertical plane facing
    forward, centred ahead of and below the shoulder."""
    return (args.cx,
            args.cy + args.radius * math.cos(phase),
            args.cz + args.radius * math.sin(phase))


def base_to_shoulder(side, p):
    """A torso-frame point (x fwd, y left, z up) in the arm's shoulder frame
    (forward, outward, up), where 'outward' is away from the body: +y for the
    left arm, -y for the right. Matches the sign `to_joint_space` expects."""
    x, y, z = p
    ox, oy, oz = SHOULDER[side]
    forward = x - ox
    up = z - oz
    outward = (y - oy) if side == "l" else -(y - oy)
    return (forward, outward, up)


def shared_offset(args, phase):
    """The forward/up offset on the shared circle at this phase.

    The circle lies in each arm's forward-up plane — it varies in x (forward)
    and z (up) with no left-right swing. That is the reachable shape for two
    hands moving in unison: this robot's shoulders barely adduct (roll only
    ~+0.08 rad inward), so any circle that swept the hands side to side would
    drive the inward-moving arm past its limit. Both hands trace the same
    forward-up circle at the same phase, each in front of its own shoulder."""
    return (args.shared_radius * math.cos(phase),
            args.shared_radius * math.sin(phase))


def plan(args, sides):
    """The joint-space path for one revolution.

    Returns a list of `(phase, per_arm, residual)` where `per_arm` maps each
    side to its own `{sh_p, sh_r, elb_p}`. Mirrored mode gives both arms the
    same values (the config's `direction` flips the right one at the motor);
    shared mode solves each arm to its own point on one torso-frame circle.
    """
    n = int(args.period * args.rate)
    path = []
    if args.shared:
        q = {"l": [0.5, 0.2, 1.2], "r": [0.5, 0.2, 1.2]}
        for k in range(n):
            phase = 2 * math.pi * k / n
            dx, dz = shared_offset(args, phase)
            per_arm = {}
            worst = 0.0
            for side in ("l", "r"):
                # Each hand in front of its own shoulder, at the shoulder's own
                # lateral position (plus an outward spread), tracing the same
                # forward-up circle at the same phase. No inward reach, so both
                # arms stay in their roll range.
                sy = SHOULDER[side][1]
                spread = args.shared_spread if side == "l" else -args.shared_spread
                target_base = (args.shared_cx + dx, sy + spread, args.shared_cz + dz)
                target = base_to_shoulder(side, target_base)
                q[side], res = ik(target, q[side], args.upper, args.lower)
                worst = max(worst, res)
                per_arm[side] = to_joint_space(q[side], args.sh_p_sign)
            if worst > 0.003:
                raise SystemExit(
                    f"IK did not converge at phase {phase:.2f}: residual {worst*1000:.1f} mm — a "
                    f"hand cannot reach the shared circle. Shrink --shared-radius or adjust --shared-cx/--shared-cz.")
            path.append((phase, per_arm, worst))
        return path

    q = [0.5, 0.2, 1.2]  # a bent-arm seed; refined by the first solve
    for k in range(n):
        phase = 2 * math.pi * k / n
        target = circle_point(args, phase)
        q, res = ik(target, q, args.upper, args.lower)
        if res > 0.003:
            raise SystemExit(f"IK did not converge at phase {phase:.2f}: residual {res*1000:.1f} mm "
                             f"— the circle is out of reach for {args.upper}+{args.lower} m")
        jq = to_joint_space(q, args.sh_p_sign)
        path.append((phase, {"l": jq, "r": jq}, res))
    return path


def check_limits(path, joints):
    worst = {}
    for _, per_arm, _ in path:
        for side in ("l", "r"):
            for key, val in per_arm[side].items():
                name = ARMS[side][key]
                if name not in joints:
                    continue
                lo, hi = joints[name]["min_rad"], joints[name]["max_rad"]
                if val < lo or val > hi:
                    worst.setdefault(name, []).append(val)
    return worst


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--panel", default="http://127.0.0.1:8085")
    ap.add_argument("--upper", type=float, default=0.25, help="upper arm length, m (retarget.yaml: 0.25)")
    ap.add_argument("--lower", type=float, default=0.25, help="forearm length to the wrist, m (retarget.yaml: 0.25)")
    ap.add_argument("--cx", type=float, default=0.30, help="circle centre, forward of the shoulder, m")
    ap.add_argument("--cy", type=float, default=0.12, help="circle centre, outward of the shoulder, m")
    ap.add_argument("--cz", type=float, default=-0.26, help="circle centre, above the shoulder, m (negative = below)")
    ap.add_argument("--radius", type=float, default=0.06, help="m")
    ap.add_argument("--shared", action="store_true",
                    help="both hands draw the same forward-up circle in unison, side by side, one in front of each shoulder")
    ap.add_argument("--shared-cx", type=float, default=0.28, help="shared circle centre, forward of the torso, m")
    ap.add_argument("--shared-cz", type=float, default=0.28, help="shared circle centre height, torso z, m (shoulder is 0.60)")
    ap.add_argument("--shared-radius", type=float, default=0.06,
                    help="shared circle radius, m (small: the elbow flex limit caps a forward-up circle at ~0.06)")
    ap.add_argument("--shared-spread", type=float, default=0.0,
                    help="push each hand this far outward of its shoulder, m (0 = directly in front of each shoulder)")
    ap.add_argument("--period", type=float, default=10.0, help="seconds per revolution")
    ap.add_argument("--revolutions", type=float, default=0.0, help="0 = forever, until Ctrl+C")
    ap.add_argument("--gripper", type=float, default=1.0,
                    help="how far each gripper folds, rad, toward the far end of its learned range; 0 disables")
    ap.add_argument("--gripper-at", type=float, default=0.35, help="where in the revolution the fold starts, 0..1")
    ap.add_argument("--gripper-ramp", type=float, default=0.20,
                    help="fold / unfold time, fraction of a revolution (the grippers are limited to 0.5 rad/s)")
    ap.add_argument("--gripper-hold", type=float, default=0.10, help="time held folded, fraction of a revolution")
    ap.add_argument("--head-yaw", type=float, default=0.25, help="head looks side to side by this much, rad; 0 disables")
    ap.add_argument("--head-pitch", type=float, default=0.20, help="head nods by this much about its base pose, rad; 0 disables")
    ap.add_argument("--rate", type=float, default=20.0, help="setpoints per second")
    ap.add_argument("--sh-p-sign", type=float, default=-1.0, choices=[1.0, -1.0],
                    help="-1: negative sh_p raises the arm forward (verified on the robot); +1 flips it")
    ap.add_argument("--arms", default="lr", help="which arms: l, r or lr")
    ap.add_argument("--approach-velocity", type=float, default=0.3, help="rad/s for the profiled move to the start")
    ap.add_argument("--settle-tol", type=float, default=0.12,
                    help="rad; how close to the start counts as arrived (servos hold a few degrees short under gravity)")
    ap.add_argument("--dry-run", action="store_true", help="print the joint path and exit without connecting")
    args = ap.parse_args()

    sides = [s for s in ("l", "r") if s in args.arms]
    if args.shared and sides != ["l", "r"]:
        raise SystemExit("--shared needs both arms; drop --arms or set --arms lr")
    path = plan(args, sides)

    # A look at the path before anything moves.
    keys = ("sh_p", "sh_r", "elb_p")
    kind = "one shared circle" if args.shared else "a mirrored circle per arm"
    print(f"{len(path)} setpoints per revolution, {args.period} s each, {kind}, arms {sides}")
    for key in keys:
        vals = [per_arm[s][key] for _, per_arm, _ in path for s in sides]
        print(f"  {key:6s} {min(vals):+.3f} .. {max(vals):+.3f} rad")
    print(f"  worst IK residual {max(r for _, _, r in path)*1000:.2f} mm")
    if args.dry_run:
        n = len(path)
        show = sides if args.shared else ["l"]
        for idx in range(0, n, max(1, n // 10)):
            phase, per_arm, _ = path[idx]
            fold = gripper_profile(idx / n, args)
            f = idx / n
            arm_txt = "  ".join(f"{s}:{k}={per_arm[s][k]:+.2f}" for s in show for k in keys)
            print(f"  phase={phase:4.2f}  {arm_txt}"
                  + f"  gripper={fold:.2f}"
                  + f"  head_y={args.head_yaw * math.sin(2 * math.pi * f):+.2f}"
                  + f"  head_p={-0.1745 + args.head_pitch * math.sin(4 * math.pi * f):+.2f}")
        return

    panel = Panel(args.panel)
    try:
        joints = panel.joints()
    except URLError as error:
        raise SystemExit(f"no panel at {args.panel}: {error}")

    names = [ARMS[s][k] for s in sides for k in ARMS[s] if ARMS[s][k] in joints]
    head_names = [n for n in ("head_y", "head_p") if n in joints]
    names += head_names
    missing = [ARMS[s][k] for s in sides for k in keys if ARMS[s][k] not in joints]
    if missing:
        raise SystemExit(f"the robot has no joint(s) {missing}")

    over = check_limits(path, joints)
    if over:
        for name, vals in over.items():
            print(f"  {name}: path reaches {min(vals):+.3f} .. {max(vals):+.3f} rad, outside "
                  f"[{joints[name]['min_rad']}, {joints[name]['max_rad']}]")
        raise SystemExit("the circle leaves a joint's envelope; shrink --radius or move the centre")

    # A gripper folds toward the far end of the range that was learned for it
    # (l_gripper -0.00 .. +2.43, r_gripper -2.47 .. +0.81 on the bench: the two
    # sides are wired opposite ways, and the description's zero is the open
    # end on both), clamped so it never reaches the limit itself.
    gripper = {}
    for s in sides:
        name = ARMS[s]["gripper"]
        if name in joints and args.gripper > 0:
            lo, hi = joints[name]["min_rad"], joints[name]["max_rad"]
            sign = 1.0 if abs(hi) >= abs(lo) else -1.0
            reach = min(args.gripper, (abs(hi) if sign > 0 else abs(lo)) * 0.9)
            gripper[name] = sign * reach
            print(f"  {name}: folds to {gripper[name]:+.2f} rad (range {lo:+.2f} .. {hi:+.2f})")

    # The head follows the arms: it looks from side to side once a revolution
    # and nods twice, around the base pose's pitch, clamped inside the learned
    # range with a margin.
    head = {}
    for name, amp, centre, cycles in (("head_y", args.head_yaw, 0.0, 1.0),
                                      ("head_p", args.head_pitch, -0.1745, 2.0)):
        if name in joints and amp > 0:
            lo, hi = joints[name]["min_rad"], joints[name]["max_rad"]
            lo, hi = lo + 0.05 * (hi - lo), hi - 0.05 * (hi - lo)
            head[name] = (amp, centre, cycles, lo, hi)
            print(f"  {name}: {centre:+.2f} ± {amp:.2f} rad, {cycles:g}x per revolution, clamped to {lo:+.2f} .. {hi:+.2f}")

    def targets_at(index):
        per_arm = path[index % len(path)][1]
        fraction = (index % len(path)) / len(path)
        t = {}
        for s in sides:
            for k in keys:
                t[ARMS[s][k]] = per_arm[s][k]
            for k in ("sh_y", "elb_y", "wrist_p"):
                if ARMS[s][k] in joints:
                    t[ARMS[s][k]] = 0.0
        fold = gripper_profile(fraction, args)
        for name, full in gripper.items():
            t[name] = full * fold
        for name, (amp, centre, cycles, lo, hi) in head.items():
            t[name] = min(hi, max(lo, centre + amp * math.sin(2 * math.pi * cycles * fraction)))
        return t

    # 1. Profiled move to the first point of the circle, and wait there.
    start = targets_at(0)
    print("moving to the start of the circle...")
    r = panel.move(start, args.approach_velocity, args.approach_velocity * 2)
    if "error" in r:
        raise SystemExit(f"move refused: {r['error']}")
    wait_settled(panel, list(start), tol=args.settle_tol, timeout=30)

    # 2. Stream the circle, forever unless told how many turns.
    forever = args.revolutions <= 0
    total = None if forever else int(args.revolutions * len(path))
    print("drawing until Ctrl+C" if forever else f"drawing {args.revolutions} revolution(s) - Ctrl+C to stop")
    dt = 1.0 / args.rate
    t0 = time.perf_counter()
    try:
        i = -1
        while total is None or i + 1 < total:
            i += 1
            r = panel.pose(targets_at(i))
            if "error" in r:
                js = panel.joints()
                stopped = [f"{n} ({js[n]['estop_reason']})" for n in names if js[n]["estop"]]
                raise SystemExit(f"pose refused at t={i*dt:.1f}s: {r['error']}"
                                 + ("; e-stop on " + ", ".join(stopped) if stopped else ""))
            if i % int(args.rate) == 0:
                js = panel.joints()
                stopped = [n for n in names if js[n]["estop"]]
                if stopped:
                    raise SystemExit("e-stop on " + ", ".join(
                        f"{n} ({js[n]['estop_reason']})" for n in stopped))
                err = lambda ns: max((abs(js[n]["position_rad"] - js[n]["target_rad"]) for n in ns), default=0.0)
                arm_names = [n for n in names if n not in gripper and n not in head_names]
                arms_err = err(arm_names)
                grip_err = err(gripper)
                head_err = err(head_names)
                print(f"  t={i*dt:5.1f}s  arm {arms_err:.3f} rad"
                      + (f", gripper {grip_err:.3f} rad" if gripper else "")
                      + (f", head {head_err:.3f} rad" if head_names else ""))
            next_at = t0 + (i + 1) * dt
            sleep = next_at - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nstopped by user; holding the last setpoint "
              "(the panel's base button, or POST /api/move {\"pose\":\"base\"}, returns it)")
        return

    # 3. Back to base, under a profile.
    print("returning to base pose...")
    r = panel.move_pose("base", args.approach_velocity, args.approach_velocity * 2)
    if "error" in r:
        print(f"  refused: {r['error']}")


if __name__ == "__main__":
    main()
