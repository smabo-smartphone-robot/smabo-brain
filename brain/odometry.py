"""Differential-drive odometry for smabo-brain.

Receives /wheel_vel messages relayed from smabo-esp32 and integrates
them into a nav_msgs/Odometry pose.  Config parameters (wheel geometry,
covariance, frame names) are kept in sync via update_config() whenever
smabo-esp32 sends a set_config reply.
"""

import math
import time

_BIG = 1e6   # variance for unmeasured DoF (z, roll, pitch)


def _build_pose_cov(cov_cfg: dict) -> list[float]:
    c = [0.0] * 36
    c[0]  = cov_cfg.get("pose_xx", 0.001)
    c[7]  = cov_cfg.get("pose_yy", 0.001)
    c[14] = _BIG
    c[21] = _BIG
    c[28] = _BIG
    c[35] = cov_cfg.get("pose_aa", 0.001)
    return c


def _build_twist_cov(cov_cfg: dict) -> list[float]:
    c = [0.0] * 36
    c[0]  = cov_cfg.get("twist_vv", 0.001)
    c[7]  = _BIG
    c[14] = _BIG
    c[21] = _BIG
    c[28] = _BIG
    c[35] = cov_cfg.get("twist_ww", 0.001)
    return c


class Odometry:
    """Stateful integrator: call update_config() on config changes,
    integrate() on each /wheel_vel message."""

    def __init__(self) -> None:
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self._esp32_cfg: dict = {}

    def update_config(self, esp32_cfg: dict) -> None:
        """Sync wheel geometry and frame names from an ESP32 set_config payload."""
        self._esp32_cfg = esp32_cfg

    def integrate(self, v_left: float, v_right: float, dt: float) -> dict | None:
        """Integrate one wheel-velocity sample into the pose.

        Parameters
        ----------
        v_left, v_right : float
            Left / right wheel speed in m/s.
        dt : float
            Integration interval in seconds.

        Returns
        -------
        dict
            nav_msgs/Odometry message ready to publish, or None if dt ≤ 0.
        """
        if dt <= 0:
            return None

        sep = (self._esp32_cfg.get("dc") or {}).get("wheel_separation", 0.15)

        d_l = v_left  * dt
        d_r = v_right * dt
        d_center = (d_l + d_r) / 2.0
        d_theta  = (d_r - d_l) / sep

        self.x     += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y     += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        vx = d_center / dt
        wz = d_theta  / dt

        return self._build_msg(vx, wz)

    def _build_msg(self, vx: float, wz: float) -> dict:
        enc = self._esp32_cfg.get("encoder") or {}
        cov = enc.get("covariance") or {}
        now = time.time()
        qz  = math.sin(self.theta / 2.0)
        qw  = math.cos(self.theta / 2.0)
        return {
            "header": {
                "stamp":    {"sec": int(now), "nanosec": int((now % 1) * 1e9)},
                "frame_id": enc.get("odom_frame", "odom"),
            },
            "child_frame_id": enc.get("base_frame", "base_link"),
            "pose": {
                "pose": {
                    "position":    {"x": self.x, "y": self.y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                },
                "covariance": _build_pose_cov(cov),
            },
            "twist": {
                "twist": {
                    "linear":  {"x": vx,  "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": wz},
                },
                "covariance": _build_twist_cov(cov),
            },
        }
