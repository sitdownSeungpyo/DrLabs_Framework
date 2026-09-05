#!/usr/bin/env python3
"""Faces through a head-mounted RealSense D435i: who is there, who is looking.

Runs only in an environment with pyrealsense2, opencv and mediapipe — on this
machine that is C:\\pj_depthcam_teleop\\imitation_upper\\.venv. demo_show.py
imports it for --face; on its own it is a viewer for checking the camera, the
head-pose signs and the "looking at the robot" timer without a robot:

    <venv>\\python.exe tools\\face_tracker.py --view

What it produces, for the show:

- Tracks. Each detected face is matched to an existing track by nearest
  centre; a track unseen for `lost_after` seconds is dropped. A dropped track
  that had been greeted is remembered by place and size for `regreet_after`
  seconds, and a new track appearing there inherits the greeting — a person
  who was briefly lost is the same person, a person who left and came back
  after that is a new one (this is the whole identity model: position, not
  face recognition).
- Looking. A face whose head pose is within `look_yaw_deg` / `look_pitch_deg`
  of the camera axis is looking at the robot; the camera is on the head and
  the head follows the face, so the axis is the robot's face. `looking_for`
  is how long that has held, with a short grace for a missed frame.
- A candidate. The nearest live track that has not been greeted (by depth
  at the nose; by box size when depth is missing). The head follows the
  candidate; when none exists the head searches.
- An observation. `observation()` is the candidate's bearing from the
  optical axis and the time of the frame it was seen in; `HeadController`
  turns those into head joint commands (latency-compensated, rate-limited —
  see its docstring). The joint signs are not known here — the show passes
  `yaw_sign` / `pitch_sign` from its calibration. `--simulate` exercises the
  controller against a model of the lag without a camera.

Head pose comes from the landmarker's facial transformation matrix. The face
forward axis is the third column of its rotation; yaw and pitch are the
angles of that axis from the camera's optical axis. Their signs were read off
the viewer, not a datasheet — see `pose_angles`.
"""

import argparse
import collections
import math
import os
import threading
import time
import urllib.request

import numpy as np

# MediaPipe's absl logging tries to phone home ("clearcut uploader") and
# reports the refusal at error level, every few seconds. Quiet it before the
# import; the landmarker's own errors still surface as exceptions.
os.environ.setdefault("GLOG_minloglevel", "3")
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")

HERE = os.path.dirname(os.path.abspath(__file__))


def _iou(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    iw = max(0, min(ax1, bx1) - max(ax0, bx0))
    ih = max(0, min(ay1, by1) - max(ay0, by0))
    inter = iw * ih
    union = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return inter / union if union > 0 else 0.0
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/face_landmarker/"
             "face_landmarker/float16/1/face_landmarker.task")
DEFAULT_MODEL = os.path.join(HERE, "models", "face_landmarker.task")

NOSE_TIP = 1  # face mesh landmark index


def download_model(path=DEFAULT_MODEL):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(f"downloading {MODEL_URL} -> {path}")
    urllib.request.urlretrieve(MODEL_URL, path)
    print(f"  {os.path.getsize(path)} bytes")


# ---------------------------------------------------------------- camera

class Camera:
    """A D435i colour + aligned depth stream, restarted when it stalls."""

    def __init__(self, width=640, height=480, fps=30, max_failures=10):
        import pyrealsense2 as rs
        self.rs = rs
        self.width, self.height, self.fps = width, height, fps
        self.max_failures = max_failures
        self.pipeline = None
        self.align = None
        self.depth_scale = 0.001
        self.fx = self.fy = self.ppx = self.ppy = None
        self.failures = 0
        self.backoff = 0.1

    def open(self):
        rs = self.rs
        self.close()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.pipeline = rs.pipeline()
        profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy, self.ppx, self.ppy = intr.fx, intr.fy, intr.ppx, intr.ppy
        self.failures = 0
        self.backoff = 0.1

    def close(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None

    def read(self):
        """(bgr, depth_m, t_ms) or None when a frame did not arrive."""
        if self.pipeline is None:
            try:
                self.open()
            except Exception as error:
                self.failures += 1
                time.sleep(self.backoff)
                self.backoff = min(2.0, self.backoff * 2)
                self.last_error = str(error)
                return None
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            frames = self.align.process(frames)
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color or not depth:
                raise RuntimeError("incomplete frameset")
        except Exception as error:
            self.failures += 1
            self.last_error = str(error)
            if self.failures >= self.max_failures:
                self.close()
            return None
        self.failures = 0
        bgr = np.asanyarray(color.get_data())
        depth_m = np.asanyarray(depth.get_data()).astype(np.float32) * self.depth_scale
        return bgr, depth_m, int(time.monotonic() * 1000)

    def bearing(self, px, py):
        """Angles (rad) of a pixel from the optical axis: +x right, +y down."""
        return math.atan((px - self.ppx) / self.fx), math.atan((py - self.ppy) / self.fy)


# ---------------------------------------------------------------- pose

def pose_angles(matrix):
    """(yaw_deg, pitch_deg) of the face's forward axis from the camera axis.

    The landmarker's transformation matrix is face-to-camera; its third
    rotation column is where the face points, in camera coordinates. Only
    the magnitudes are relied on (the look test is symmetric); which way is
    positive has not been checked against a person on this camera.
    """
    r = np.asarray(matrix, dtype=float)[:3, :3]
    f = r[:, 2]
    yaw = math.degrees(math.atan2(f[0], f[2]))
    pitch = math.degrees(math.atan2(f[1], f[2]))
    return yaw, pitch


# ---------------------------------------------------------------- tracks

class Track:
    __slots__ = ("id", "bbox", "centre", "smooth_centre", "size", "distance_m", "yaw", "pitch",
                 "first_seen", "last_seen", "looking_since", "look_lost_at", "greeted", "hits")

    def __init__(self, tid, t):
        self.id = tid
        self.bbox = (0, 0, 0, 0)
        self.centre = (0.0, 0.0)
        self.smooth_centre = None
        self.size = 0.0
        self.distance_m = None
        self.yaw = 0.0
        self.pitch = 0.0
        self.first_seen = t
        self.last_seen = t
        self.looking_since = None
        self.look_lost_at = None
        self.greeted = False
        self.hits = 0

    def looking_for(self, t):
        return 0.0 if self.looking_since is None else t - self.looking_since

    def as_dict(self, t):
        return {"id": self.id, "bbox": self.bbox, "centre": self.centre, "size": self.size,
                "distance_m": self.distance_m, "yaw": self.yaw, "pitch": self.pitch,
                "age_s": t - self.first_seen, "looking_for": self.looking_for(t),
                "greeted": self.greeted}


class FaceTracker:
    def __init__(self, model=DEFAULT_MODEL, num_faces=3, look_seconds=3.0, look_yaw_deg=20.0,
                 look_pitch_deg=20.0, lost_after=1.0, regreet_after=10.0, look_grace=0.5,
                 width=640, height=480, fps=30):
        self.model = model
        self.num_faces = num_faces
        self.look_seconds = look_seconds
        self.look_yaw_deg = look_yaw_deg
        self.look_pitch_deg = look_pitch_deg
        self.lost_after = lost_after
        self.regreet_after = regreet_after
        self.look_grace = look_grace
        self.camera = Camera(width, height, fps)
        self.lock = threading.Lock()
        self.tracks = {}
        self.greeted_places = []  # (centre, size, t_dropped)
        self.next_id = 1
        self.frame = None
        self.frame_t = 0.0
        self.fps_measured = 0.0
        self.error = ""
        self.detect_ms = 0.0
        self.running = False
        self.thread = None
        self.landmarker = None
        self._jpeg = None
        self._jpeg_t = 0.0

    # -- lifecycle

    def start(self):
        if not os.path.exists(self.model):
            raise FileNotFoundError(f"no face model at {self.model}; run face_tracker.py --download-model")
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
        self.mp = mp
        options = vision.FaceLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.model),
            running_mode=vision.RunningMode.VIDEO,
            num_faces=self.num_faces,
            output_facial_transformation_matrixes=True,
            min_face_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        self.landmarker = vision.FaceLandmarker.create_from_options(options)
        self.running = True
        self.thread = threading.Thread(target=self._run, name="face-tracker", daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=3)
        self.camera.close()
        if self.landmarker is not None:
            self.landmarker.close()

    # -- the loop

    def _run(self):
        last_ts = -1
        tick = time.monotonic()
        frames = 0
        while self.running:
            got = self.camera.read()
            if got is None:
                with self.lock:
                    self.error = getattr(self.camera, "last_error", "no frame")
                    self._expire(time.monotonic())
                continue
            bgr, depth_m, ts = got
            if ts <= last_ts:
                ts = last_ts + 1
            last_ts = ts
            t0 = time.perf_counter()
            rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
            try:
                result = self.landmarker.detect_for_video(image, ts)
            except Exception as error:
                with self.lock:
                    self.error = f"landmarker: {error}"
                continue
            detections = self._detections(result, depth_m, bgr.shape[1], bgr.shape[0])
            now = time.monotonic()
            with self.lock:
                self._update(detections, now)
                self.frame = bgr
                self.frame_t = now
                self.error = ""
                self.detect_ms = (time.perf_counter() - t0) * 1000
                frames += 1
                if now - tick >= 1.0:
                    self.fps_measured = frames / (now - tick)
                    frames = 0
                    tick = now

    def _detections(self, result, depth_m, w, h):
        out = []
        matrices = result.facial_transformation_matrixes or []
        for i, landmarks in enumerate(result.face_landmarks or []):
            xs = [lm.x * w for lm in landmarks]
            ys = [lm.y * h for lm in landmarks]
            x0, x1 = max(0, int(min(xs))), min(w - 1, int(max(xs)))
            y0, y1 = max(0, int(min(ys))), min(h - 1, int(max(ys)))
            if x1 <= x0 or y1 <= y0:
                continue
            nose = landmarks[NOSE_TIP]
            nx, ny = int(nose.x * w), int(nose.y * h)
            patch = depth_m[max(0, ny - 2):ny + 3, max(0, nx - 2):nx + 3]
            valid = patch[(patch > 0.15) & (patch < 6.0)]
            distance = float(np.median(valid)) if valid.size else None
            yaw, pitch = pose_angles(matrices[i]) if i < len(matrices) else (0.0, 0.0)
            det = {"bbox": (x0, y0, x1, y1), "centre": ((x0 + x1) / 2, (y0 + y1) / 2),
                   "size": float(max(x1 - x0, y1 - y0)), "distance_m": distance,
                   "yaw": yaw, "pitch": pitch}
            # The landmarker can report one face twice with num_faces > 1;
            # two boxes that mostly overlap are one person (seen on the bench:
            # two tracks 3 cm apart in depth, greeted one after the other).
            if any(_iou(det["bbox"], other["bbox"]) > 0.4 for other in out):
                continue
            out.append(det)
        return out

    def _update(self, detections, t):
        # Greedy nearest-centre matching, closest pairs first.
        pairs = []
        for d_index, det in enumerate(detections):
            for track in self.tracks.values():
                dist = math.hypot(det["centre"][0] - track.centre[0], det["centre"][1] - track.centre[1])
                if dist <= max(80.0, 0.6 * max(det["size"], track.size)):
                    pairs.append((dist, d_index, track.id))
        pairs.sort()
        used_d, used_t = set(), set()
        for _, d_index, tid in pairs:
            if d_index in used_d or tid in used_t:
                continue
            used_d.add(d_index)
            used_t.add(tid)
            self._observe(self.tracks[tid], detections[d_index], t)
        for d_index, det in enumerate(detections):
            if d_index in used_d:
                continue
            track = Track(self.next_id, t)
            self.next_id += 1
            self._observe(track, det, t)
            # Someone we already greeted, briefly lost — or a second box on a
            # person we are greeting right now?
            places = list(self.greeted_places) + [
                (tr.centre, tr.size, t) for tr in self.tracks.values() if tr.greeted]
            for centre, size, _ in places:
                near = math.hypot(det["centre"][0] - centre[0], det["centre"][1] - centre[1])
                if near <= 1.5 * max(size, det["size"]):
                    track.greeted = True
                    break
            self.tracks[track.id] = track
        self._expire(t)

    def _observe(self, track, det, t):
        track.bbox = det["bbox"]
        track.centre = det["centre"]
        # A few frames of averaging on the centre the head is aimed at; the
        # landmark box wobbles by a few pixels frame to frame.
        if track.smooth_centre is None:
            track.smooth_centre = det["centre"]
        else:
            track.smooth_centre = tuple(0.6 * s + 0.4 * c for s, c in zip(track.smooth_centre, det["centre"]))
        track.size = det["size"]
        if det["distance_m"] is not None:
            track.distance_m = det["distance_m"]
        track.yaw = det["yaw"]
        track.pitch = det["pitch"]
        track.last_seen = t
        track.hits += 1
        looking = abs(track.yaw) <= self.look_yaw_deg and abs(track.pitch) <= self.look_pitch_deg
        if looking:
            if track.looking_since is None:
                track.looking_since = t
            track.look_lost_at = None
        else:
            if track.look_lost_at is None:
                track.look_lost_at = t
            elif t - track.look_lost_at > self.look_grace:
                track.looking_since = None

    def _expire(self, t):
        for tid in [tid for tid, tr in self.tracks.items() if t - tr.last_seen > self.lost_after]:
            track = self.tracks.pop(tid)
            if track.greeted:
                self.greeted_places.append((track.centre, track.size, t))
        self.greeted_places = [p for p in self.greeted_places if t - p[2] <= self.regreet_after]
        # A track that is not seen this instant keeps its look timer only through the grace.
        for track in self.tracks.values():
            if t - track.last_seen > self.look_grace and track.looking_since is not None:
                track.looking_since = None
                track.look_lost_at = None

    # -- queries (all take the lock; cheap)

    def candidate(self):
        """The nearest un-greeted face — but the one being followed keeps
        its place unless it has been gone 0.7 s or somebody is clearly
        (20 %) closer, so the head does not flick between two people."""
        with self.lock:
            t = time.monotonic()
            live = [tr for tr in self.tracks.values() if not tr.greeted and tr.hits >= 3]
            if not live:
                self.current_id = None
                return None

            def distance(tr):
                return tr.distance_m if tr.distance_m is not None else 10.0 / max(tr.size, 1.0) * 100

            nearest = min(live, key=distance)
            current = self.tracks.get(getattr(self, "current_id", None))
            if (current is not None and not current.greeted and t - current.last_seen < 0.7
                    and distance(nearest) > 0.8 * distance(current)):
                pick = current
            else:
                pick = nearest
            self.current_id = pick.id
            return pick.as_dict(t)

    def greet_ready(self):
        c = self.candidate()
        if c is not None and c["looking_for"] >= self.look_seconds:
            return c
        return None

    def mark_greeted(self, tid):
        with self.lock:
            track = self.tracks.get(tid)
            if track is not None:
                track.greeted = True
                track.looking_since = None

    def snapshot(self):
        with self.lock:
            t = time.monotonic()
            return {"tracks": [tr.as_dict(t) for tr in self.tracks.values()],
                    "fps": round(self.fps_measured, 1), "detect_ms": round(self.detect_ms, 1),
                    "frame_age_s": round(t - self.frame_t, 2) if self.frame_t else None,
                    "error": self.error, "greeted_places": len(self.greeted_places)}

    # -- the head

    def observation(self):
        """The candidate as the head controller wants it: the bearing of its
        (smoothed) centre from the optical axis and the time of the frame it
        was seen in. None when there is nobody to follow."""
        cand = self.candidate()
        if cand is None or not self.camera.fx:
            return None
        with self.lock:
            track = self.tracks.get(cand["id"])
            if track is None:
                return None
            centre, seen = track.smooth_centre, track.last_seen
        return {"id": cand["id"], "frame_t": seen, "bearing": self.camera.bearing(*centre)}


    # -- pictures

    def annotated(self):
        import cv2
        with self.lock:
            if self.frame is None:
                return None
            frame = self.frame.copy()
            t = time.monotonic()
            tracks = [tr.as_dict(t) for tr in self.tracks.values()]
            fps, ms, err = self.fps_measured, self.detect_ms, self.error
        cand = self.candidate()
        cand_id = cand["id"] if cand else None
        for tr in tracks:
            x0, y0, x1, y1 = tr["bbox"]
            colour = (0, 0, 255) if tr["greeted"] else (0, 255, 0) if tr["id"] == cand_id else (255, 200, 0)
            cv2.rectangle(frame, (x0, y0), (x1, y1), colour, 2)
            dist = f"{tr['distance_m']:.2f}m" if tr["distance_m"] else "?m"
            label = f"#{tr['id']} {dist} yaw{tr['yaw']:+.0f} pit{tr['pitch']:+.0f}"
            if tr["greeted"]:
                label += " greeted"
            elif tr["looking_for"] > 0:
                label += f" look {tr['looking_for']:.1f}s"
            cv2.putText(frame, label, (x0, max(12, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
            if tr["looking_for"] > 0 and not tr["greeted"]:
                frac = min(1.0, tr["looking_for"] / self.look_seconds)
                cv2.rectangle(frame, (x0, y1 + 3), (x0 + int((x1 - x0) * frac), y1 + 8), (0, 255, 0), -1)
        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        status = f"{fps:.0f} fps  {ms:.0f} ms  faces {len(tracks)}" + (f"  ERR {err}" if err else "")
        cv2.putText(frame, status, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return frame

    def jpeg(self, max_hz=5.0, quality=70):
        import cv2
        now = time.monotonic()
        if self._jpeg is not None and now - self._jpeg_t < 1.0 / max_hz:
            return self._jpeg
        frame = self.annotated()
        if frame is None:
            return self._jpeg
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if ok:
            self._jpeg = buf.tobytes()
            self._jpeg_t = now
        return self._jpeg


# ---------------------------------------------------------------- viewer


class HeadController:
    """Turns face bearings into head joint commands that a servo can follow
    without hunting.

    The first version added a fraction of the bearing to the command on every
    tick. With ~200 ms between a head movement and its appearance in a
    detection (exposure, detection, the panel's 10 Hz loop, the servo), the
    head had been corrected several more times before the first correction
    showed — it overshot, then swung back, on every face. This one:

    - forms an *absolute* target once per new frame: the command that was in
      force when the frame was taken (from a short history, `latency` back)
      plus the bearing seen in that frame. One frame, one correction.
    - ignores bearings inside a deadband, so a still face is a still head.
    - low-passes the target (`tau`) and moves the command toward it under a
      velocity and an acceleration ceiling, with the stopping-distance rule
      so it does not overshoot the target.
    - when the face is lost, holds for `hold_seconds`, then sweeps the room
      on a sine that starts from where the head is.

    Pure arithmetic on the numbers it is given: `--simulate` runs it against
    a model of the camera lag and the servo.
    """

    def __init__(self, yaw_sign, pitch_sign, base_yaw, base_pitch, yaw_range, pitch_range,
                 latency=0.35, gain=0.7, tau=0.2, max_vel=0.5, max_acc=2.0, deadband_deg=1.5,
                 hold_seconds=1.5, search_amplitude=0.5, search_period=10.0):
        self.yaw_sign, self.pitch_sign = yaw_sign, pitch_sign
        self.base = (base_yaw, base_pitch)
        self.ranges = (yaw_range, pitch_range)
        self.latency, self.tau = latency, tau
        # Less than the whole bearing per frame: the latency is only known
        # roughly, and the remainder is picked up by the next frame.
        self.gain = gain
        self.max_vel, self.max_acc = max_vel, max_acc
        self.deadband = math.radians(deadband_deg)
        self.hold_seconds = hold_seconds
        self.search_amplitude, self.search_period = search_amplitude, search_period
        self.pos = [base_yaw, base_pitch]      # the command, as sent
        self.vel = [0.0, 0.0]
        self.target = [base_yaw, base_pitch]   # smoothed
        self.raw = [base_yaw, base_pitch]      # last absolute target
        self.history = collections.deque()     # (t, yaw, pitch) commands
        self.last_frame_t = -1.0
        self.last_seen_t = None
        self.search_start = None
        self.search_phase = 0.0
        self.t_prev = None
        self.mode = "search"
        self.following = None

    def reset(self, yaw, pitch):
        """Adopt where the head actually is (after a restart)."""
        self.pos = [yaw, pitch]
        self.vel = [0.0, 0.0]
        self.target = [yaw, pitch]
        self.raw = [yaw, pitch]
        self.history.clear()

    def command_at(self, t):
        """The command that was in force at time t."""
        best = None
        for entry in self.history:
            if entry[0] <= t:
                best = entry
            else:
                break
        if best is None:
            best = self.history[0] if self.history else (t, self.pos[0], self.pos[1])
        return best[1], best[2]

    def update(self, t, observation):
        """One tick. Returns (yaw, pitch) to send."""
        dt = 0.05 if self.t_prev is None else min(0.1, max(0.005, t - self.t_prev))
        self.t_prev = t

        if observation is not None and observation["frame_t"] > self.last_frame_t:
            self.last_frame_t = observation["frame_t"]
            self.last_seen_t = t
            self.following = observation["id"]
            y0, p0 = self.command_at(observation["frame_t"] - self.latency)
            bx, by = observation["bearing"]
            if abs(bx) >= self.deadband:
                self.raw[0] = y0 + self.yaw_sign * self.gain * bx
            if abs(by) >= self.deadband:
                self.raw[1] = p0 + self.pitch_sign * self.gain * by
            self.mode = "track"
            self.search_start = None
        elif self.last_seen_t is not None and t - self.last_seen_t < self.hold_seconds:
            self.mode = "hold"
        else:
            if self.search_start is None:
                # Start the sweep from where the head is, not from base.
                self.search_start = t
                ratio = (self.pos[0] - self.base[0]) / self.search_amplitude if self.search_amplitude else 0.0
                self.search_phase = math.asin(max(-1.0, min(1.0, ratio)))
                self.following = None
            phase = self.search_phase + 2 * math.pi * (t - self.search_start) / self.search_period
            self.raw[0] = self.base[0] + self.search_amplitude * math.sin(phase)
            self.raw[1] = self.base[1]
            self.mode = "search"

        alpha = 1.0 - math.exp(-dt / self.tau) if self.tau > 0 else 1.0
        for i in range(2):
            lo, hi = self.ranges[i]
            self.raw[i] = min(hi, max(lo, self.raw[i]))
            self.target[i] += (self.raw[i] - self.target[i]) * alpha
            err = self.target[i] - self.pos[i]
            # Fast enough to get there, slow enough to stop there.
            v_want = math.copysign(min(self.max_vel, math.sqrt(2 * self.max_acc * abs(err))), err)
            dv = max(-self.max_acc * dt, min(self.max_acc * dt, v_want - self.vel[i]))
            self.vel[i] += dv
            self.pos[i] += self.vel[i] * dt
            if self.pos[i] < lo or self.pos[i] > hi:
                self.pos[i] = min(hi, max(lo, self.pos[i]))
                self.vel[i] = 0.0
        self.history.append((t, self.pos[0], self.pos[1]))
        while self.history and t - self.history[0][0] > 2.0:
            self.history.popleft()
        return self.pos[0], self.pos[1]

    def summary(self):
        return {"yaw": round(self.pos[0], 3), "pitch": round(self.pos[1], 3),
                "target_yaw": round(self.target[0], 3), "target_pitch": round(self.target[1], 3),
                "mode": self.mode, "following": self.following}


def simulate(latency_true=0.2, noise_rad=0.005, servo_tau=0.08, seconds=6.0, **kw):
    """The controller against a model of the loop it lives in: a face at a
    fixed bearing, a servo that follows the command with a lag through a
    10 Hz sample-and-hold, and a 30 fps camera whose frames show the head
    as it was `latency_true` ago. Returns (overshoot fraction, settle time,
    peak-to-peak command jitter once settled)."""
    import random
    face_yaw = 0.5                       # rad, where the face really is
    ctl = HeadController(1.0, 1.0, 0.0, 0.0, (-2.0, 2.0), (-2.0, 2.0), **kw)
    head_true = 0.0
    held = 0.0                           # the panel's 10 Hz sample of the command
    history = []                         # (t, head_true)
    step = 0.005
    t = 0.0
    next_tick = 0.0
    next_frame = 0.0
    next_hold = 0.0
    obs = None
    cmd = 0.0
    peak = 0.0
    settled_at = None
    tail = []
    while t < seconds:
        if t >= next_hold:
            held = cmd
            next_hold += 0.1
        head_true += (held - head_true) * (1 - math.exp(-step / servo_tau))
        history.append((t, head_true))
        if t >= next_frame:
            t_seen = t - latency_true
            seen = next((h for tt, h in reversed(history) if tt <= t_seen), 0.0)
            bearing = face_yaw - seen + random.gauss(0, noise_rad)
            obs = {"id": 1, "frame_t": t, "bearing": (bearing, 0.0)}
            next_frame += 1 / 30
        if t >= next_tick:
            cmd, _ = ctl.update(t, obs)
            next_tick += 0.05
        peak = max(peak, head_true)
        if settled_at is None and abs(head_true - face_yaw) < math.radians(2):
            settled_at = t
        elif settled_at is not None and abs(head_true - face_yaw) >= math.radians(2):
            settled_at = None
        if t > seconds - 2.0:
            tail.append(cmd)
        t += step
    overshoot = max(0.0, peak - face_yaw) / face_yaw
    jitter = (max(tail) - min(tail)) if tail else 0.0
    return overshoot, settled_at, jitter

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--download-model", action="store_true")
    ap.add_argument("--view", action="store_true", help="show the annotated stream in a window")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = until q/Ctrl+C)")
    ap.add_argument("--look-seconds", type=float, default=3.0)
    ap.add_argument("--look-yaw-deg", type=float, default=20.0)
    ap.add_argument("--look-pitch-deg", type=float, default=20.0)
    ap.add_argument("--regreet-after", type=float, default=10.0)
    ap.add_argument("--greet-when-ready", action="store_true",
                    help="viewer only: mark the candidate greeted when its look timer completes")
    ap.add_argument("--simulate", action="store_true",
                    help="no camera: run the head controller against a model of the lag and the servo")
    args = ap.parse_args()
    if args.simulate:
        worst = 0
        for lag in (0.1, 0.2, 0.3):
            overshoot, settled, jitter = simulate(latency_true=lag)
            # 0.5 rad at 0.5 rad/s with the acceleration ramps is ~1.3 s on its
            # own; the low-pass adds a little. Sweep of 2026-09-05 (docs/SHOW.md).
            ok = overshoot < 0.10 and settled is not None and settled < 2.5 and jitter < math.radians(0.5)
            worst += 0 if ok else 1
            print(f"lag {lag:.1f} s: overshoot {overshoot*100:4.1f} %  settled "
                  f"{'never' if settled is None else f'{settled:.2f} s'}  jitter {math.degrees(jitter):.2f} deg  "
                  f"{'ok' if ok else 'FAIL'}")
        raise SystemExit(worst)
    if args.download_model:
        download_model(args.model)
        if not args.view:
            return
    tracker = FaceTracker(model=args.model, look_seconds=args.look_seconds,
                          look_yaw_deg=args.look_yaw_deg, look_pitch_deg=args.look_pitch_deg,
                          regreet_after=args.regreet_after)
    tracker.start()
    print("tracker started; q or Ctrl+C to stop")
    started = time.monotonic()
    last_print = 0.0
    try:
        import cv2
        while True:
            if args.seconds and time.monotonic() - started > args.seconds:
                break
            snap = tracker.snapshot()
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                cand = tracker.candidate()
                line = f"{snap['fps']:5.1f} fps {snap['detect_ms']:5.1f} ms faces {len(snap['tracks'])}"
                for tr in snap["tracks"]:
                    d = f"{tr['distance_m']:.2f}" if tr["distance_m"] else "?"
                    line += (f" | #{tr['id']} {d}m yaw{tr['yaw']:+5.0f} pit{tr['pitch']:+5.0f}"
                             f" look{tr['looking_for']:4.1f}{' G' if tr['greeted'] else ''}")
                if cand:
                    line += f" | candidate #{cand['id']}"
                if snap["error"]:
                    line += f" | {snap['error']}"
                print(line, flush=True)
                ready = tracker.greet_ready()
                if ready and args.greet_when_ready:
                    print(f"  -> would greet #{ready['id']}; marking greeted")
                    tracker.mark_greeted(ready["id"])
            if args.view:
                frame = tracker.annotated()
                if frame is not None:
                    cv2.imshow("face_tracker", frame)
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop()
        if args.view:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
