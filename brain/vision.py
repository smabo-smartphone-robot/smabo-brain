"""Vision processing for smabo — pure, transport-agnostic logic.

Like ``brain.odometry``, this module knows nothing about WebSockets, ROS, time
sources or message envelopes.  It exposes:

  - **detectors** (``detect_*``) that turn a decoded BGR image into a list of
    plain detection dicts (pixel-space bbox + class/score) and, for AR / QR
    modes, the recognized strings;
  - a **Detection2DArray builder** (``to_detection2darray``) that lays the
    detections out in the ``vision_msgs/Detection2DArray`` shape (used verbatim
    as rosbridge JSON by smabo-brain and as a real ROS 2 message by
    smabo-brain-ros);
  - **policies** that turn one chosen detection into actuator-agnostic geometry
    (``bbox_to_direction``) and then into a ``/look_at`` pose or a
    ``/servo/command`` trajectory.

Keeping CV + geometry here lets both transports (WS relay = smabo-brain,
ROS 2 = smabo-brain-ros) reuse exactly the same maths.  OpenCV is imported
lazily so a vision-less deployment of smabo-brain still runs.
"""

import math

# OpenCV / numpy are only needed when a detector actually runs.  Import lazily
# so importing brain.vision never breaks a deployment that does not use vision.
try:
    import cv2
    import numpy as np
    HAVE_CV2 = True
except Exception:  # pragma: no cover - optional dependency
    cv2 = None
    np = None
    HAVE_CV2 = False


# ---------------------------------------------------------------------------
# Configuration / defaults
# ---------------------------------------------------------------------------

#: Recognized vision modes.  "off" disables processing.
MODES = ("off", "aruco", "color", "face", "qr")

#: Named colours → list of (lower, upper) HSV ranges (OpenCV HSV: H 0–179).
#: Red wraps the hue circle so it needs two ranges.
COLOR_HSV = {
    "red":    [((0, 110, 90), (10, 255, 255)), ((170, 110, 90), (179, 255, 255))],
    "orange": [((11, 120, 110), (22, 255, 255))],
    "yellow": [((23, 90, 110), (35, 255, 255))],
    "green":  [((36, 80, 70), (85, 255, 255))],
    "cyan":   [((86, 80, 70), (100, 255, 255))],
    "blue":   [((101, 90, 70), (130, 255, 255))],
    "purple": [((131, 70, 70), (160, 255, 255))],
}

#: Default ArUco dictionary (used unless config overrides it).
DEFAULT_ARUCO_DICT = "DICT_4X4_50"

#: Default horizontal field of view of the phone camera (deg).  Phone cameras
#: vary widely; expose this in the vision config so it can be tuned per device.
DEFAULT_HFOV_DEG = 60.0


def _parse_rgb(val):
    """Normalize a colour value to ``[r, g, b]`` ints (0–255), or ``None``.

    Accepts a hex string ``"#RRGGBB"``/``"RRGGBB"`` or a 3-element list/tuple.
    """
    if val is None or val == "":
        return None
    if isinstance(val, str):
        s = val.lstrip("#")
        if len(s) == 6:
            try:
                return [int(s[0:2], 16), int(s[2:4], 16), int(s[4:6], 16)]
            except ValueError:
                return None
        return None
    if isinstance(val, (list, tuple)) and len(val) == 3:
        try:
            return [max(0, min(255, int(v))) for v in val]
        except (ValueError, TypeError):
            return None
    return None


def _rgb_to_hex(rgb) -> str:
    r, g, b = (int(max(0, min(255, c))) for c in rgb)
    return f"#{r:02x}{g:02x}{b:02x}"


class VisionConfig:
    """Resolved vision settings (from the ``/vision/config`` control message)."""

    def __init__(self, raw: dict | None = None):
        raw = raw or {}
        self.enabled = bool(raw.get("enabled", False))
        self.mode = raw.get("mode", "off")
        if self.mode not in MODES:
            self.mode = "off"
        self.color = raw.get("color", "red")
        # Arbitrary target colour from a palette (hex "#RRGGBB" or [r,g,b]).
        # When set it takes precedence over the named `color` preset.
        self.color_rgb = _parse_rgb(raw.get("color_rgb"))
        # Colour matching range (RGB mode): hue half-window + saturation/value
        # floors (OpenCV H 0–179, S/V 0–255). Lower floors accept paler/darker.
        self.color_hue_tol = int(raw.get("color_hue_tol", 12))
        self.color_s_min = int(raw.get("color_s_min", 70))
        self.color_v_min = int(raw.get("color_v_min", 60))
        # Minimum detection size as a fraction of the frame area (rejects specks).
        self.min_area_frac = float(raw.get("min_area_frac", 0.0008))
        # Frame capture rate (fps) the web client sends to the brain for
        # detection. Higher = more responsive look_at/servo/drive (useful for a
        # mobile robot) at the cost of bandwidth + CPU. Clamped to a sane range.
        self.capture_fps = float(raw.get("capture_fps", 5.0))
        if self.capture_fps < 1.0:
            self.capture_fps = 1.0
        elif self.capture_fps > 30.0:
            self.capture_fps = 30.0
        self.speak = bool(raw.get("speak", False))
        self.aruco_dict = raw.get("aruco_dict", DEFAULT_ARUCO_DICT)
        # Marker to prefer when driving look_at/servo (None = largest area).
        tm = raw.get("target_marker_id", None)
        self.target_marker_id = str(tm) if tm not in (None, "") else None
        self.hfov_deg = float(raw.get("hfov_deg", DEFAULT_HFOV_DEG))
        # Which joints the neck/servo policy drives, and their tuning.
        tj = raw.get("target_joints") or {}
        self.pan_joint = tj.get("pan", "head_pan")
        # Tilt is NOT a default joint (head_tilt was removed from the default
        # robot); only drive it when a tilt joint is configured explicitly.
        self.tilt_joint = tj.get("tilt", "")
        self.pan_sign = float(tj.get("pan_sign", 1.0))
        self.tilt_sign = float(tj.get("tilt_sign", 1.0))
        self.servo_gain = float(tj.get("gain", 1.0))
        # Which behaviors a detection drives (each independently toggled by web).
        bh = raw.get("behaviors") or {}
        self.do_look_at = bool(bh.get("look_at", True))
        self.do_servo = bool(bh.get("servo", False))
        self.do_drive = bool(bh.get("drive", False))
        # Drive (mobile-robot follow) tuning.
        dr = raw.get("drive") or {}
        self.drive_target_frac = float(dr.get("target_area_frac", 0.10))
        self.drive_k_ang = float(dr.get("k_ang", 1.5))
        self.drive_k_lin = float(dr.get("k_lin", 2.0))
        self.drive_max_ang = float(dr.get("max_ang", 1.0))
        self.drive_max_lin = float(dr.get("max_lin", 0.20))
        self.drive_deadzone = float(dr.get("deadzone", 0.02))

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled, "mode": self.mode, "color": self.color,
            "color_rgb": self.color_rgb, "color_hue_tol": self.color_hue_tol,
            "color_s_min": self.color_s_min, "color_v_min": self.color_v_min,
            "min_area_frac": self.min_area_frac,
            "capture_fps": self.capture_fps,
            "speak": self.speak, "aruco_dict": self.aruco_dict,
            "target_marker_id": self.target_marker_id, "hfov_deg": self.hfov_deg,
            "target_joints": {
                "pan": self.pan_joint, "tilt": self.tilt_joint,
                "pan_sign": self.pan_sign, "tilt_sign": self.tilt_sign,
                "gain": self.servo_gain,
            },
            "behaviors": {
                "look_at": self.do_look_at, "servo": self.do_servo, "drive": self.do_drive,
            },
            "drive": {
                "target_area_frac": self.drive_target_frac,
                "k_ang": self.drive_k_ang, "k_lin": self.drive_k_lin,
                "max_ang": self.drive_max_ang, "max_lin": self.drive_max_lin,
                "deadzone": self.drive_deadzone,
            },
        }


def merge_config(base: dict, patch: dict) -> dict:
    """Deep-merge a partial vision-config ``patch`` onto ``base`` (both in
    ``VisionConfig.to_dict()`` shape) and return a new dict.

    Nested dict keys (``target_joints`` / ``behaviors`` / ``drive``) merge per
    field so a partial ``/vision/config`` message can change e.g. only
    ``behaviors.servo`` without clobbering the other behaviour flags. Scalar
    keys are replaced. Inputs are left untouched so the same ``base`` (the
    startup default) can be reused. This is pure so both transports
    (smabo-brain relay, smabo-brain-ros) apply overrides identically.
    """
    out = {k: (dict(v) if isinstance(v, dict) else v) for k, v in base.items()}
    for k, v in (patch or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            merged = dict(out[k])
            merged.update(v)
            out[k] = merged
        else:
            out[k] = v
    return out


# ---------------------------------------------------------------------------
# Image decode
# ---------------------------------------------------------------------------

#: Maximum width used for detection.  Frames wider than this are downscaled
#: before running the detector so CV operations are proportionally faster.
#: 640 is a practical balance: ArUco / colour work well; upscale the bbox
#: coordinates back afterward.  Set to 0 to disable (use native resolution).
DETECT_MAX_W = 640


def resize_for_detection(bgr):
    """Downscale *bgr* to at most ``DETECT_MAX_W`` pixels wide for detection.

    Returns ``(detect_bgr, inv_scale)`` where ``inv_scale`` multiplies
    detection-image coordinates back to original-image coordinates (1.0 when
    no resize was needed).  The min-size threshold stays consistent because
    ``sqrt(min_area_frac * W * H)`` scales linearly with image dimensions.
    """
    if not HAVE_CV2 or DETECT_MAX_W <= 0:
        return bgr, 1.0
    h, w = bgr.shape[:2]
    if w <= DETECT_MAX_W:
        return bgr, 1.0
    scale = DETECT_MAX_W / w
    resized = cv2.resize(bgr, (DETECT_MAX_W, int(h * scale)))
    return resized, 1.0 / scale


# ---------------------------------------------------------------------------
# Detection: a detector returns a list of dicts shaped like:
#   {"class_id": str, "score": float, "cx": float, "cy": float, "w": float, "h": float}
# (all pixel coordinates), plus optionally a list of recognized strings.
# ---------------------------------------------------------------------------

def _det(class_id, score, cx, cy, w, h):
    return {"class_id": str(class_id), "score": float(score),
            "cx": float(cx), "cy": float(cy), "w": float(w), "h": float(h)}


#: ``aruco_dict`` sentinel: scan every dictionary in ``ARUCO_ALL_DICTS``.
ARUCO_ALL = "ALL"

#: Dictionaries scanned in "ALL" mode. Each ``_1000`` superset already contains
#: the smaller ``_50/_100/_250`` ids of its family, so this short list covers the
#: common 4x4–7x7 ArUco families plus the original ArUco set and AprilTag 36h11.
ARUCO_ALL_DICTS = (
    "DICT_4X4_1000", "DICT_5X5_1000", "DICT_6X6_1000", "DICT_7X7_1000",
    "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_36h11",
)

#: Cache detector objects per dictionary (creating one per frame is wasteful,
#: and "ALL" mode would otherwise rebuild several every frame).
_ARUCO_CACHE: dict = {}


def _get_aruco_detector(dict_name):
    """Return a cached ArUco detector for ``dict_name`` (or ``None`` if unknown)."""
    if dict_name in _ARUCO_CACHE:
        return _ARUCO_CACHE[dict_name]
    d = getattr(cv2.aruco, dict_name, None)
    if d is None:
        _ARUCO_CACHE[dict_name] = None
        return None
    adict = cv2.aruco.getPredefinedDictionary(d)
    # OpenCV ≥4.7 ArucoDetector, with a fallback to the legacy free function.
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(adict, cv2.aruco.DetectorParameters())
    else:
        det = adict
    _ARUCO_CACHE[dict_name] = det
    return det


def _detect_aruco_one(bgr, dict_name):
    """Detect markers of a single dictionary → list of detection dicts."""
    det = _get_aruco_detector(dict_name)
    if det is None:
        return []
    if hasattr(det, "detectMarkers"):
        corners, ids, _ = det.detectMarkers(bgr)
    else:  # legacy API: det is the dictionary
        corners, ids, _ = cv2.aruco.detectMarkers(bgr, det)
    out = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.flatten()):
        pts = c.reshape(-1, 2)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        out.append(_det(int(i), 1.0, (x0 + x1) / 2.0, (y0 + y1) / 2.0,
                        x1 - x0, y1 - y0))
    return out


def _dedup_detections(dets):
    """Drop near-coincident detections (same marker found by several dicts).

    Keeps the larger bbox when two centres are within half the smaller width.
    """
    kept = []
    for d in sorted(dets, key=lambda d: d["w"] * d["h"], reverse=True):
        dup = False
        for k in kept:
            dist = ((d["cx"] - k["cx"]) ** 2 + (d["cy"] - k["cy"]) ** 2) ** 0.5
            if dist < 0.5 * min(d["w"], k["w"]):
                dup = True
                break
        if not dup:
            kept.append(d)
    return kept


def detect_aruco(bgr, dict_name=DEFAULT_ARUCO_DICT):
    """Detect ArUco markers → (detections, [marker_id strings]).

    ``dict_name`` may be a single dictionary, or ``"ALL"`` (case-insensitive,
    also "ANY"/"AUTO") to scan every dictionary in ``ARUCO_ALL_DICTS`` and merge
    the results (deduplicated). "ALL" trades CPU for not having to pick a dict.
    """
    if str(dict_name).upper() in (ARUCO_ALL, "ANY", "AUTO"):
        names = ARUCO_ALL_DICTS
    else:
        names = (dict_name,)
    found = []
    for dn in names:
        found.extend(_detect_aruco_one(bgr, dn))
    if len(names) > 1:
        found = _dedup_detections(found)
    strings = []
    for d in found:
        if d["class_id"] not in strings:
            strings.append(d["class_id"])
    return found, strings


def _hsv_ranges_from_rgb(color_rgb, hue_tol=12, s_min=70, v_min=60):
    """Build OpenCV-HSV ``(lo, hi)`` inRange tuples around a target RGB.

    Converts the RGB to a hue centre and opens a ``±hue_tol`` window, splitting
    into two ranges when the window wraps the 0/179 hue boundary (as for reds).
    Saturation/value floors reject washed-out / dark pixels.
    """
    r, g, b = (int(max(0, min(255, c))) for c in color_rgb)
    px = np.uint8([[[b, g, r]]])                       # a single BGR pixel
    h, _s, _v = (int(x) for x in cv2.cvtColor(px, cv2.COLOR_BGR2HSV)[0][0])
    lo_h, hi_h = h - hue_tol, h + hue_tol
    if lo_h < 0:
        return [((0, s_min, v_min), (hi_h, 255, 255)),
                ((180 + lo_h, s_min, v_min), (179, 255, 255))]
    if hi_h > 179:
        return [((lo_h, s_min, v_min), (179, 255, 255)),
                ((0, s_min, v_min), (hi_h - 180, 255, 255))]
    return [((lo_h, s_min, v_min), (hi_h, 255, 255))]


def detect_color(bgr, color="red", color_rgb=None, hue_tol=12, s_min=70, v_min=60):
    """Detect blobs of a target colour → detections (largest blobs first).

    ``color_rgb`` (hex/list, see :func:`_parse_rgb`) selects an arbitrary colour
    from a palette and takes precedence; otherwise the named ``color`` preset
    (``COLOR_HSV``) is used. ``hue_tol``/``s_min``/``v_min`` set the matching
    range (RGB mode); ``min_area_frac`` drops blobs smaller than that fraction of
    the frame. ``class_id`` is the hex colour (RGB mode) or the preset name.
    """
    if color_rgb is not None:
        ranges = _hsv_ranges_from_rgb(color_rgb, hue_tol=hue_tol,
                                      s_min=s_min, v_min=v_min)
        label = _rgb_to_hex(color_rgb)
    else:
        ranges = COLOR_HSV.get(color)
        label = color
    if not ranges:
        return []
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area_total = float(bgr.shape[0] * bgr.shape[1])
    detections = []
    for cnt in cnts:
        a = cv2.contourArea(cnt)
        if a < 1.0:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        detections.append(_det(label, min(1.0, a / area_total * 50.0),
                               x + w / 2.0, y + h / 2.0, w, h))
    return detections


def detect_qr(bgr):
    """Detect QR codes → (detections, [decoded content strings])."""
    qr = cv2.QRCodeDetector()
    detections, strings = [], []
    try:
        ok, texts, points, _ = qr.detectAndDecodeMulti(bgr)
    except Exception:
        ok = False
    if not ok or points is None:
        return detections, strings
    for text, quad in zip(texts, points):
        pts = quad.reshape(-1, 2)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        detections.append(_det(text or "qr", 1.0,
                               (x0 + x1) / 2.0, (y0 + y1) / 2.0, x1 - x0, y1 - y0))
        if text:
            strings.append(text)
    return detections, strings


_FACE_CASCADE = None


def detect_face(bgr):
    """Detect faces (Haar cascade) → detections (class_id='face')."""
    global _FACE_CASCADE
    if _FACE_CASCADE is None:
        path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        _FACE_CASCADE = cv2.CascadeClassifier(path)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    faces = _FACE_CASCADE.detectMultiScale(gray, 1.2, 5, minSize=(40, 40))
    detections = []
    for (x, y, w, h) in faces:
        detections.append(_det("face", 1.0, x + w / 2.0, y + h / 2.0, w, h))
    return detections


def run_detector(bgr, cfg: VisionConfig):
    """Dispatch to the configured detector.

    Returns ``(detections, strings, (W, H))``.  ``strings`` are the recognized
    AR ids / QR contents (empty for color/face).
    """
    h, w = bgr.shape[:2]
    if cfg.mode == "aruco":
        d, s = detect_aruco(bgr, cfg.aruco_dict)
    elif cfg.mode == "color":
        d, s = detect_color(bgr, cfg.color, cfg.color_rgb, cfg.color_hue_tol,
                            cfg.color_s_min, cfg.color_v_min), []
    elif cfg.mode == "qr":
        d, s = detect_qr(bgr)
    elif cfg.mode == "face":
        d, s = detect_face(bgr), []
    else:
        d, s = [], []

    # Minimum-size filter: the shorter side of the bbox must be >= the side of
    # the reference square shown in the UI (sqrt(min_area_frac * W * H)).
    # Using the shorter side makes the filter match the visual reference rectangle:
    # "if the object's shorter dimension is larger than the box, it passes."
    # Area-based filtering was misleading for elongated objects (a 200×10 strip
    # could have less area than the reference square while looking much larger).
    if cfg.min_area_frac > 0 and d:
        min_dim = math.sqrt(cfg.min_area_frac * float(w * h))
        d = [x for x in d if min(x["w"], x["h"]) >= min_dim]
        if s:
            survivors = {x["class_id"] for x in d}
            s = [t for t in s if t in survivors]
    return d, s, (w, h)


# ---------------------------------------------------------------------------
# Detection2DArray (vision_msgs) — pure value → value
# ---------------------------------------------------------------------------

def to_detection2darray(detections, frame_id, stamp, img_wh=None):
    """Build a ``vision_msgs/Detection2DArray``-shaped dict from detections.

    ``stamp`` is a ``{"sec","nanosec"}`` dict supplied by the transport.
    ``img_wh`` (optional ``(W,H)``) is echoed in the array header frame_id-less
    way via a non-standard ``source_img_width/height`` hint so consumers that
    only see the message can normalize bbox centers if needed.
    """
    header = {"stamp": stamp, "frame_id": frame_id}
    out = {"header": header, "detections": []}
    if img_wh:
        out["source_img_width"], out["source_img_height"] = int(img_wh[0]), int(img_wh[1])
    for d in detections:
        out["detections"].append({
            "header": header,
            "bbox": {
                "center": {"position": {"x": d["cx"], "y": d["cy"]}, "theta": 0.0},
                "size_x": d["w"], "size_y": d["h"],
            },
            "results": [{
                "hypothesis": {"class_id": d["class_id"], "score": d["score"]},
            }],
        })
    return out


def detections_from_detection2darray(msg: dict):
    """Inverse of ``to_detection2darray`` — used by the policy consumers."""
    out = []
    for d in (msg or {}).get("detections", []):
        bbox = d.get("bbox") or {}
        ctr = (bbox.get("center") or {}).get("position") or {}
        res = (d.get("results") or [{}])[0].get("hypothesis") or {}
        out.append(_det(res.get("class_id", ""), res.get("score", 1.0),
                        ctr.get("x", 0.0), ctr.get("y", 0.0),
                        bbox.get("size_x", 0.0), bbox.get("size_y", 0.0)))
    wh = (msg.get("source_img_width"), msg.get("source_img_height"))
    return out, (wh if all(v is not None for v in wh) else None)


# ---------------------------------------------------------------------------
# Policy: pick a target, project to a direction, drive look_at / servo
# ---------------------------------------------------------------------------

def pick_target(detections, target_marker_id=None):
    """Pick the detection to act on: a preferred id if present, else largest area."""
    if not detections:
        return None
    if target_marker_id is not None:
        for d in detections:
            if d["class_id"] == str(target_marker_id):
                return d
    return max(detections, key=lambda d: d["w"] * d["h"])


def bbox_to_direction(cx, cy, w, h, hfov_deg):
    """Project a pixel position to a direction in the robot frame (REP-103).

    Returns a unit-ish vector ``(x, y, z)`` = (forward, left, up).  The
    horizontal FOV is given; the vertical FOV is derived from the image aspect.
    """
    if w <= 0 or h <= 0:
        return (1.0, 0.0, 0.0)
    u_n = (cx - w / 2.0) / (w / 2.0)        # image x: +right, range ~[-1,1]
    v_n = (cy - h / 2.0) / (h / 2.0)        # image y: +down
    hfov = math.radians(hfov_deg)
    vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * (h / w))
    az = u_n * (hfov / 2.0)                  # target right → +y (look right in image → look left REP-103)
    el = -v_n * (vfov / 2.0)                # up positive    (target down → -z)
    x = math.cos(el) * math.cos(az)
    y = math.cos(el) * math.sin(az)
    z = math.sin(el)
    return (x, y, z)


def direction_to_look_at(direction, frame_id, stamp):
    """Build a ``geometry_msgs/PoseStamped`` (position = look-at direction)."""
    x, y, z = direction
    return {
        "header": {"stamp": stamp, "frame_id": frame_id},
        "pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }


def direction_to_servo(direction, cfg: VisionConfig, stamp, joints_available=None):
    """Build a ``trajectory_msgs/JointTrajectory`` aiming the neck at ``direction``.

    Only emits joints listed in ``cfg`` that also exist in ``joints_available``
    (when provided), so the policy adapts to the robot's actual servos.
    """
    x, y, z = direction
    pan = cfg.pan_sign * math.atan2(y, x) * cfg.servo_gain
    tilt = cfg.tilt_sign * math.atan2(z, math.hypot(x, y)) * cfg.servo_gain
    names, positions = [], []
    for joint, value in ((cfg.pan_joint, pan), (cfg.tilt_joint, tilt)):
        if not joint:
            continue
        if joints_available is not None and joint not in joints_available:
            continue
        names.append(joint)
        positions.append(value)
    if not names:
        return None
    return {
        "header": {"stamp": stamp, "frame_id": ""},
        "joint_names": names,
        "points": [{
            "positions": positions, "velocities": [],
            "time_from_start": {"sec": 0, "nanosec": 200_000_000},
        }],
    }


_STOP_TWIST = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
               "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}


def to_cmd_vel(target, img_wh, cfg: VisionConfig):
    """Make the mobile robot follow ``target`` → ``geometry_msgs/Twist`` dict.

    Turns to keep the target centered (``angular.z``) and approaches/recedes to
    hold a target apparent size (``linear.x``, from the bbox area fraction as a
    distance proxy).  Returns a stop twist when there is no target.
    """
    if target is None or not img_wh:
        return dict(_STOP_TWIST)
    w, h = img_wh
    if w <= 0 or h <= 0:
        return dict(_STOP_TWIST)
    u_n = (target["cx"] - w / 2.0) / (w / 2.0)            # +right
    area_frac = (target["w"] * target["h"]) / float(w * h)

    ang = max(-cfg.drive_max_ang, min(cfg.drive_max_ang, -cfg.drive_k_ang * u_n))

    err = cfg.drive_target_frac - area_frac              # >0: too far → forward
    lin = cfg.drive_k_lin * err
    if abs(u_n) > 0.5:                                    # don't drive in until facing
        lin = min(lin, 0.0)
    if abs(err) < cfg.drive_deadzone:
        lin = 0.0
    lin = max(-cfg.drive_max_lin, min(cfg.drive_max_lin, lin))

    return {"linear": {"x": lin, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": ang}}

