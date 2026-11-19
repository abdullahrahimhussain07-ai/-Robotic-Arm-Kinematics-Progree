import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

plt.rcParams["figure.facecolor"] = "white"
plt.rcParams["axes.facecolor"] = "white"
plt.rcParams["savefig.facecolor"] = "white"


class ThreeDOFArm:
    def __init__(self, d1=0.20, l2=0.35, l3=0.30, joint_limits_deg=None, min_z=0.0):
        self.d1 = d1
        self.l2 = l2
        self.l3 = l3
        self.min_z = min_z
        self.joint_limits_deg = joint_limits_deg or {
            "q1": (-180.0, 180.0),
            "q2": (-90.0, 135.0),
            "q3": (-150.0, 150.0),
        }

    def link_positions(self, q1, q2, q3):
        p0 = np.array([0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, self.d1])
        r2 = self.l2 * math.cos(q2)
        z2 = self.d1 + self.l2 * math.sin(q2)
        p2 = np.array([r2 * math.cos(q1), r2 * math.sin(q1), z2])
        r3 = r2 + self.l3 * math.cos(q2 + q3)
        z3 = z2 + self.l3 * math.sin(q2 + q3)
        p3 = np.array([r3 * math.cos(q1), r3 * math.sin(q1), z3])
        return np.vstack([p0, p1, p2, p3])

    def forward_kinematics(self, q1, q2, q3):
        ee = self.link_positions(q1, q2, q3)[-1]
        return float(ee[0]), float(ee[1]), float(ee[2])

    def inverse_kinematics(self, x, y, z, elbow_up=False):
        if z < self.min_z - 1e-9:
            raise ValueError(f"Target z={z:.3f} m violates z >= {self.min_z:.3f} m.")
        r = math.hypot(x, y)
        s = z - self.d1
        max_reach = self.l2 + self.l3
        min_reach = abs(self.l2 - self.l3)
        dist = math.hypot(r, s)
        if dist > max_reach + 1e-9 or dist < min_reach - 1e-9:
            raise ValueError(f"Target ({x:.3f}, {y:.3f}, {z:.3f}) out of reach.")
        q1 = math.atan2(y, x)
        cos_q3 = max(-1.0, min(1.0, (r**2 + s**2 - self.l2**2 - self.l3**2) / (2.0 * self.l2 * self.l3)))
        sin_q3_mag = math.sqrt(max(0.0, 1.0 - cos_q3**2))
        q3 = -math.atan2(sin_q3_mag, cos_q3) if elbow_up else math.atan2(sin_q3_mag, cos_q3)
        phi = math.atan2(s, r)
        psi = math.atan2(self.l3 * math.sin(q3), self.l2 + self.l3 * math.cos(q3))
        q2 = phi - psi
        return q1, q2, q3

    def validate_trajectory(self, q_deg, path_xyz):
        warnings = []
        for idx, name in enumerate(["q1", "q2", "q3"]):
            lo, hi = self.joint_limits_deg[name]
            vals = q_deg[:, idx]
            if np.any(vals < lo) or np.any(vals > hi):
                warnings.append(f"{name} out of limits [{lo} deg, {hi} deg]")
        if np.any(path_xyz[:, 2] < self.min_z):
            warnings.append(f"Path dips below z_min={self.min_z:.3f} m")
        min_sin_q3 = float(np.min(np.abs(np.sin(np.radians(q_deg[:, 2])))))
        if min_sin_q3 < 0.08:
            warnings.append(f"Near elbow singularity: min |sin(q3)| = {min_sin_q3:.3f}")
        return warnings


def quintic_segment(p_start, p_end, steps):
    u = np.linspace(0.0, 1.0, steps)
    s = 10 * (u**3) - 15 * (u**4) + 6 * (u**5)
    return p_start[None, :] + np.outer(s, p_end - p_start)


def build_pick_place_cycle(pick_pt, place_pt, clearance, steps_per_leg):
    z_off = np.array([0.0, 0.0, clearance])
    nodes = [
        (pick_pt + z_off, 0),
        (pick_pt, 0),
        (pick_pt + z_off, 1),
        (place_pt + z_off, 1),
        (place_pt, 1),
        (place_pt + z_off, 0),
    ]
    pts_list, grip_list = [], []
    for i in range(len(nodes) - 1):
        p_a, g_a = nodes[i]
        p_b, _ = nodes[i + 1]
        seg = quintic_segment(p_a, p_b, steps_per_leg)
        g_seg = np.full(steps_per_leg, g_a, dtype=int)
        if i > 0:
            seg, g_seg = seg[1:], g_seg[1:]
        pts_list.append(seg)
        grip_list.append(g_seg)
    return np.vstack(pts_list), np.concatenate(grip_list)


def generate_multi_point_trajectory(pick_bin, place_targets, clearance=0.12, steps_per_leg=35):
    p_pick = np.array(pick_bin, dtype=float)
    all_pts, all_grip = [], []
    for idx, target in enumerate(place_targets):
        seg_pts, seg_grip = build_pick_place_cycle(p_pick, np.array(target, dtype=float), clearance, steps_per_leg)
        if idx > 0:
            seg_pts, seg_grip = seg_pts[1:], seg_grip[1:]
        all_pts.append(seg_pts)
        all_grip.append(seg_grip)
    return np.vstack(all_pts), np.concatenate(all_grip)


def plot_workspace_envelope(ax3d, arm, alpha=0.06):
    r_max = arm.l2 + arm.l3
    u = np.linspace(0, 2 * np.pi, 28)
    v = np.linspace(0, np.pi / 2, 16)
    xs = r_max * np.outer(np.cos(u), np.sin(v))
    ys = r_max * np.outer(np.sin(u), np.sin(v))
    zs = arm.d1 + r_max * np.outer(np.ones_like(u), np.cos(v))
    ax3d.plot_surface(xs, ys, zs, color="slategray", alpha=alpha, linewidth=0, shade=False)


def plot_arm_pose(ax3d, arm, q_rad, color, label):
    pts = arm.link_positions(*q_rad)
    ax3d.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color=color, lw=2.3, markersize=4.5, label=label)


def main():
    arm = ThreeDOFArm(d1=0.20, l2=0.35, l3=0.30, min_z=0.0)
    pick_bin = (0.35, 0.15, 0.08)
    place_targets = [(0.20, -0.35, 0.14), (0.40, -0.15, 0.10)]
    steps_per_leg = 35

    path, gripper = generate_multi_point_trajectory(pick_bin, place_targets, clearance=0.12, steps_per_leg=steps_per_leg)
    joint_traj_rad, fk_verify = [], []
    for pt in path:
        q1, q2, q3 = arm.inverse_kinematics(pt[0], pt[1], pt[2], elbow_up=False)
        joint_traj_rad.append((q1, q2, q3))
        fk_verify.append(arm.forward_kinematics(q1, q2, q3))

    joint_traj_rad = np.array(joint_traj_rad)
    joint_traj_deg = np.degrees(joint_traj_rad)
    fk_verify = np.array(fk_verify)
    t = np.linspace(0.0, 8.0, len(path))
    dt = t[1] - t[0]
    q_vel_deg = np.gradient(joint_traj_deg, dt, axis=0)
    q_acc_deg = np.gradient(q_vel_deg, dt, axis=0)

    max_err = np.max(np.linalg.norm(path - fk_verify, axis=1))
    warnings = arm.validate_trajectory(joint_traj_deg, path)
    print(f"Points: {len(path)} | Max FK/IK error: {max_err:.3e} m | Warnings: {len(warnings)}")

    fig = plt.figure(figsize=(16, 9), facecolor="white")
    gs = GridSpec(4, 2, width_ratios=[1.15, 1.0], wspace=0.28, hspace=0.45)

    ax3d = fig.add_subplot(gs[:, 0], projection="3d", facecolor="white")
    plot_workspace_envelope(ax3d, arm)
    ax3d.plot(path[:, 0], path[:, 1], path[:, 2], color="navy", lw=2.0, label="EE Path")
    ax3d.scatter(*pick_bin, color="limegreen", s=75, edgecolor="black", label="Pick Bin")
    ax3d.scatter(*place_targets[0], color="crimson", s=70, edgecolor="black", label="Drop A")
    ax3d.scatter(*place_targets[1], color="darkorchid", s=70, edgecolor="black", label="Drop B")
    plot_arm_pose(ax3d, arm, tuple(joint_traj_rad[steps_per_leg]), "seagreen", "Arm @ Pick")
    plot_arm_pose(ax3d, arm, tuple(joint_traj_rad[4 * steps_per_leg]), "firebrick", "Arm @ Drop A")
    plot_arm_pose(ax3d, arm, tuple(joint_traj_rad[len(path) - 1 - steps_per_leg]), "indigo", "Arm @ Drop B")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title("3-DOF Manipulator Multi-Point Pick & Place")
    ax3d.legend(loc="upper left", fontsize=8, facecolor="white")

    ax_pos = fig.add_subplot(gs[0, 1], facecolor="white")
    ax_pos.plot(t, joint_traj_deg[:, 0], label="q1 Yaw")
    ax_pos.plot(t, joint_traj_deg[:, 1], label="q2 Shoulder")
    ax_pos.plot(t, joint_traj_deg[:, 2], label="q3 Elbow")
    ax_pos.set_ylabel("Angle (deg)")
    ax_pos.set_title("Joint Trajectories & Gripper State")
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend(loc="upper right", fontsize=7.5, ncol=3, facecolor="white")

    ax_vel = fig.add_subplot(gs[1, 1], sharex=ax_pos, facecolor="white")
    ax_vel.plot(t, q_vel_deg[:, 0], label="dq1/dt")
    ax_vel.plot(t, q_vel_deg[:, 1], label="dq2/dt")
    ax_vel.plot(t, q_vel_deg[:, 2], label="dq3/dt")
    ax_vel.set_ylabel("Vel (deg/s)")
    ax_vel.grid(True, alpha=0.3)

    ax_acc = fig.add_subplot(gs[2, 1], sharex=ax_pos, facecolor="white")
    ax_acc.plot(t, q_acc_deg[:, 0], label="d2q1/dt2")
    ax_acc.plot(t, q_acc_deg[:, 1], label="d2q2/dt2")
    ax_acc.plot(t, q_acc_deg[:, 2], label="d2q3/dt2")
    ax_acc.set_ylabel("Acc (deg/s^2)")
    ax_acc.grid(True, alpha=0.3)

    ax_grip = fig.add_subplot(gs[3, 1], sharex=ax_pos, facecolor="white")
    ax_grip.step(t, gripper, where="post", color="teal", lw=2.0)
    ax_grip.set_yticks([0, 1])
    ax_grip.set_yticklabels(["Open", "Closed"])
    ax_grip.set_ylim(-0.2, 1.35)
    ax_grip.set_xlabel("Time (s)")
    ax_grip.set_ylabel("Gripper")
    ax_grip.grid(True, alpha=0.3)

    plt.savefig("pick_and_place_trace.png", dpi=200, bbox_inches="tight", facecolor="white")
    print("Saved white-background graph to pick_and_place_trace.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
