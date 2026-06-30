
"""Plot humanoid stack demo outputs.

Usage:
  python3 humanoid_system/visualization/plot_state.py outputs/flat_ground
"""

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path):
    # DictReader 会把 CSV 第一行表头作为字段名，这样后面可以用 r["px"] 这类名字读取列。
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main():
    # 命令行参数是单个场景输出目录，例如 build/outputs/flat_ground。
    # 不传参数时默认读取平地场景。
    scenario_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "build/outputs/flat_ground")

    # trajectory.csv：估计器输出的 base 轨迹、速度、姿态和协方差 trace。
    # contact_timeline.csv：左右脚接触状态和概率。
    # semantic_grid.csv：语义占据栅格。
    traj = read_csv(scenario_dir / "trajectory.csv")
    contact = read_csv(scenario_dir / "contact_timeline.csv")
    grid = read_csv(scenario_dir / "semantic_grid.csv")

    # 轨迹图只取 x 和 z，是为了直接观察机器人是否向前移动，以及高度是否随地形变化。
    t = [float(r["t"]) for r in traj]
    px = [float(r["px"]) for r in traj]
    pz = [float(r["pz"]) for r in traj]
    com_x = [float(r["com_x"]) for r in traj] if traj and "com_x" in traj[0] else []
    com_z = [float(r["com_z"]) for r in traj] if traj and "com_z" in traj[0] else []

    # cov_trace 是协方差对角线之和，可粗略表示估计不确定性。
    cov = [float(r["cov_trace"]) for r in traj]

    # 接触状态是 0/1 序列，用 step 图更符合“状态保持直到下一次变化”的含义。
    ct = [float(r["t"]) for r in contact]
    left = [int(r["left"]) for r in contact]
    right = [int(r["right"]) for r in contact]
    left_slip = [int(r["left_slip"]) for r in contact] if contact and "left_slip" in contact[0] else []
    right_slip = [int(r["right_slip"]) for r in contact] if contact and "right_slip" in contact[0] else []

    # 只绘制置信度较高的栅格，避免 unknown/低置信度格子把图铺满。
    # x/y 是栅格坐标，不是米制 world 坐标；颜色用 occupancy 表示占据概率。
    xs = [int(r["x"]) for r in grid if float(r["confidence"]) > 0.15]
    ys = [int(r["y"]) for r in grid if float(r["confidence"]) > 0.15]
    occ = [float(r["occupancy"]) for r in grid if float(r["confidence"]) > 0.15]

    # 三行图：
    #   1. base x-z 轨迹；
    #   2. 左右脚接触时间线 + 协方差 trace；
    #   3. 语义占据栅格散点图。
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)
    axes[0].plot(px, pz)
    if com_x and com_z:
        # CoM 轨迹比 base 轨迹更接近稳定性判据；两者一起画，方便观察质心是否随支撑区合理移动。
        axes[0].plot(com_x, com_z, label="CoM", alpha=0.75)
        axes[0].legend()
    axes[0].set_title("Estimated base trajectory")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("z [m]")

    axes[1].step(ct, left, where="post", label="left")
    axes[1].step(ct, right, where="post", label="right")
    if left_slip:
        # 滑移曲线是 0/1 序列；用虚线叠加在接触时间线上，便于看出“接触但不可信”的时刻。
        axes[1].step(ct, left_slip, where="post", label="left slip", linestyle="--", alpha=0.75)
    if right_slip:
        axes[1].step(ct, right_slip, where="post", label="right slip", linestyle="--", alpha=0.75)
    axes[1].plot(t, cov, label="cov trace", alpha=0.55)
    axes[1].set_title("Contact timeline and uncertainty")
    axes[1].legend()

    sc = axes[2].scatter(xs, ys, c=occ, s=4, cmap="magma")
    axes[2].set_title("Semantic occupancy grid")
    axes[2].set_aspect("equal")
    fig.colorbar(sc, ax=axes[2], label="occupancy")

    out = scenario_dir / "visualization.png"
    fig.savefig(out, dpi=160)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
