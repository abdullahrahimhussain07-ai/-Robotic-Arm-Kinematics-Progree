import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


class ThreeDOFArm:
    """3-DOF (R-R-R: Base Yaw, Shoulder Pitch, Elbow Pitch) spatial manipulator."""

    def __init__(
        self,
        d1: float = 0.20,
        l2: float = 0.35,
        l3: float = 0.30,
        joint_limits_deg: dict[str, tuple[float, float]] | None = None,
        min_z: float = 0.0,
    ):
        self.d1 = d1
        self.l2 = l2
        self.l3 = l3
        self.min_z = min_z
        self.joint_limits_deg = joint_limits_deg or {
            "q1": (-180.0, 180.0),
            "q2": (-90.0, 135.0),
            "q3": (-150.0, 150.0),
        }

    def link_positions(self, q1: float, q2: float, q3: float) -> np.ndarray:
        """Return 3D coordinates of [BaseTop, Shoulder, Elbow, EndEffector] as (4, 3)."""
        p0 = np.array([0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, self.d1])

        r2 = self.l2 * math.cos(q2)
        z2 = self.d1 + self.l2 * math.sin(q2)
        p2 = np.array([r2 * math.cos(q1), r2 * math.sin(q1), z2])

        r3 = r2 + self.l3 * math.cos(q2 + q3)
        z3 = z2 + self.l3 * math.sin(q2 + q3)
        p3 = np.array([r3 * math.cos(q1), r3 * math.sin(q1), z3])

        return np.vstack([p0, p1, p2, p3])

    def forward_kinematics(self, q1: float, q2: float, q3: float) -> tuple[float, float, float]:
        """Compute 3D end-effector (x, y, z) from joint angles (radians)."""
        ee = self.link_positions(q1, q2, q3)[-1]
        return float(ee[0]), float(ee[1]), float(ee[2])

    def inverse_kinematics(
        self, x: float, y: float, z: float, elbow_up: bool = False
    ) -> tuple[float, float, float]:
        """Solve closed-form analytical IK for target (x, y, z) in meters."""
        if z < self.min_z - 1e-9:
            raise ValueError(f"Target z={z:.3f} m violates minimum height limit z >= {self.min_z:.3f} m.")

        r = math.hypot(x, y)
        s = z - self.d1

        max_reach = self.l2 + self.l3
        min_reach = abs(self.l2 - self.l3)
        dist = math.hypot(r, s)

        if dist > max_reach + 1e-9 or dist < min_reach - 1e-9:
            raise ValueError(
                f"Target ({x:.3f}, {y:.3f}, {z:.3f}) reach {dist:.3f} m is outside "
                f"workspace [{min_reach:.3f}, {max_reach:.3f}] m."
            )

        q1 = math.atan2(y, x)

        cos_q3 = (r**2 + s**2 - self.l2**2 - self.l3**2) / (2.0 * self.l2 * self.l3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        sin_q3_mag = math.sqrt(max(0.0, 1.0 - cos_q3**2))
        q3 = -math.atan2(sin_q3_mag, cos_q3) if elbow_up else math.atan2(sin_q3_mag, cos_q3)

        phi = math.atan2(s, r)
        psi = math.atan2(self.l3 * math.sin(q3), self.l2 + self.l3 * math.cos(q3))
        q2 = phi - psi

        return q1, q2, q3

    def validate_trajectory(self, q_deg: np.ndarray, path_xyz: np.ndarray) -> list[str]:
        """Check joint limits, floor clearance, and proximity to elbow singularities."""
        warnings: list[str] = []
        names = ["q1", "q2", "q3"]

        for idx, name in enumerate(names):
            lo, hi = self.joint_limits_deg[name]
            vals = q_deg[:, idx]
            if np.any(vals < lo) or np.any(vals > hi):
                warnings.append(
                    f"{name} out of limits [{lo}°, {hi}°] (range: [{vals.min():.1f}°, {vals.max():.1f}°])"
                )

        if np.any(path_xyz[:, 2] < self.min_z):
            warnings.append(f"Path dips below z_min={self.min_z:.3f} m (min z={path_xyz[:, 2].min():.3f} m)")

        q3_rad = np.radians(q_deg[:, 2])
        min_sin_q3 = float(np.min(np.abs(np.sin(q3_rad))))
        if min_sin_q3 < 0.08:
            warnings.append(f"Near elbow singularity: min |sin(q3)| = {min_sin_q3:.3f}")

        return warnings


def quintic_segment(p_start: np.ndarray, p_end: np.ndarray, steps: int) -> np.ndarray:
    """Smooth minimum-jerk (quintic) interpolation between two 3D points."""
    u = np.linspace(0.0, 1.0, steps)
    s = 10 * (u**3) - 15 * (u**4) + 6 * (u**5)
    return p_start[None, :] + np.outer(s, p_end - p_start)


def build_pick_place_cycle(
    pick_pt: np.ndarray,
    place_pt: np.ndarray,
    clearance: float,
    steps_per_leg: int,
    gripper_closed_at_pick: bool,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate one pick -> lift -> transit -> place -> retract segment and gripper flag (1=closed, 0=open)."""
    z_off = np.array([0.0, 0.0, clearance])
    nodes = [
        (pick_pt + z_off, 0),
        (pick_pt, 0),
        (pick_pt + z_off, 1 if gripper_closed_at_pick else 0),
        (place_pt + z_off, 1 if gripper_closed_at_pick else 0),
        (place_pt, 1 if gripper_closed_at_pick else 0),
        (place_pt + z_off, 0),
    ]

    pts_list = []
    grip_list = []
    for i in range(len(nodes) - 1):
        p_a, g_a = nodes[i]
        p_b, _ = nodes[i + 1]
        seg = quintic_segment(p_a, p_b, steps_per_leg)
        g_seg = np.full(steps_per_leg, g_a, dtype=int)
        if i > 0:
            seg = seg[1:]
            g_seg = g_seg[1:]
        pts_list.append(seg)
        grip_list.append(g_seg)

    return np.vstack(pts_list), np.concatenate(grip_list)


def generate_multi_point_trajectory(
    pick_bin: tuple[float, float, float],
    place_targets: list[tuple[float, float, float]],
    clearance: float = 0.12,
    steps_per_leg: int = 35,
) -> tuple[np.ndarray, np.ndarray]:
    """Chain multiple pick-and-place cycles across dynamic spatial coordinates."""
    p_pick = np.array(pick_bin, dtype=float)
    all_pts = []
    all_grip = []

    for idx, target in enumerate(place_targets):
        p_place = np.array(target, dtype=float)
        seg_pts, seg_grip = build_pick_place_cycle(
            p_pick, p_place, clearance=clearance, steps_per_leg=steps_per_leg, gripper_closed_at_pick=True
        )
        if idx > 0:
            seg_pts = seg_pts[1:]
            seg_grip = seg_grip[1:]
        all_pts.append(seg_pts)
        all_grip.append(seg_grip)

    return np.vstack(all_pts), np.concatenate(all_grip)


def plot_workspace_envelope(ax3d, arm: ThreeDOFArm, alpha: float = 0.05):
    """Draw outer reachable workspace hemisphere above z=0."""
    r_max = arm.l2 + arm.l3
    u = np.linspace(0, 2 * np.pi, 28)
    v = np.linspace(0, np.pi / 2, 16)
    xs = r_max * np.outer(np.cos(u), np.sin(v))
    ys = r_max * np.outer(np.sin(u), np.sin(v))
    zs = arm.d1 + r_max * np.outer(np.ones_like(u), np.cos(v))
    ax3d.plot_surface(xs, ys, zs, color="slategray", alpha=alpha, linewidth=0, shade=False)


def plot_arm_pose(ax3d, arm: ThreeDOFArm, q_rad: tuple[float, float, float], color: str, label: str):
    """Plot 3D manipulator links and joint markers for a given joint configuration."""
    pts = arm.link_positions(*q_rad)
    ax3d.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color=color, lw=2.3, markersize=4.5, label=label)


def main():
    arm = ThreeDOFArm(d1=0.20, l2=0.35, l3=0.30, min_z=0.0)

    pick_bin = (0.35, 0.15,
