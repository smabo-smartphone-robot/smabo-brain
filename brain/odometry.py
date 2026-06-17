"""Differential-drive odometry for smabo-brain.

Pure, transport-agnostic integrator: it knows nothing about WebSockets, ROS,
time sources, or message formats.  ``integrate()`` returns plain pose/twist
values so each transport can build its own message:

  - smabo-brain (WebSocket)  → rosbridge JSON, wall-clock stamp (relay.py)
  - smabo-brain-ros (rclpy)  → nav_msgs/Odometry + tf, ROS-clock stamp

Keeping time/format out of here is what lets brain-ros wrap brain by simply
importing this module.  Config parameters (wheel geometry, covariance, frame
names) come from the ESP32 config snapshot via update_config().
"""

import math

_BIG = 1e6   # variance for unmeasured DoF (z, roll, pitch)


def build_pose_covariance(cov_cfg: dict) -> list[float]:
    """Build the 36-element (6x6 row-major) pose covariance for nav_msgs/Odometry.

    Pure value→value; shared by every transport that emits nav_msgs/Odometry.
    """
    c = [0.0] * 36
    c[0]  = cov_cfg.get("pose_xx", 0.001)
    c[7]  = cov_cfg.get("pose_yy", 0.001)
    c[14] = _BIG
    c[21] = _BIG
    c[28] = _BIG
    c[35] = cov_cfg.get("pose_aa", 0.001)
    return c


def build_twist_covariance(cov_cfg: dict) -> list[float]:
    """Build the 36-element (6x6 row-major) twist covariance for nav_msgs/Odometry."""
    c = [0.0] * 36
    c[0]  = cov_cfg.get("twist_vv", 0.001)
    c[7]  = _BIG
    c[14] = _BIG
    c[21] = _BIG
    c[28] = _BIG
    c[35] = cov_cfg.get("twist_ww", 0.001)
    return c


class Odometry:
    """Stateful integrator returning raw pose/twist values (no message, no time).

    Call update_config() on config changes and integrate() on each /wheel_vel
    sample; the caller turns the returned values into whatever message its
    transport needs.
    """

    def __init__(self) -> None:
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self._esp32_cfg: dict = {}

    def update_config(self, esp32_cfg: dict) -> None:
        """Sync wheel geometry / covariance / frame names from an ESP32 config."""
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
        dict or None
            Raw pose/twist values plus covariance and frame names::

                {"x", "y", "theta", "vx", "wz",
                 "cov", "odom_frame", "base_frame"}

            or None if dt <= 0.  Time-stamping and message building are left to
            the transport layer (see relay.py for the WebSocket version).
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

        enc = self._esp32_cfg.get("encoder") or {}
        return {
            "x":          self.x,
            "y":          self.y,
            "theta":      self.theta,
            "vx":         d_center / dt,
            "wz":         d_theta  / dt,
            "cov":        enc.get("covariance") or {},
            "odom_frame": enc.get("odom_frame", "odom"),
            "base_frame": enc.get("base_frame", "base_link"),
        }
