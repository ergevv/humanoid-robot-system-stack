
"""Plot humanoid stack demo outputs.

Usage:
  python3 humanoid_system/visualization/plot_state.py outputs/flat_ground
"""

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main():
    scenario_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "build/outputs/flat_ground")
    traj = read_csv(scenario_dir / "trajectory.csv")
    contact = read_csv(scenario_dir / "contact_timeline.csv")
    grid = read_csv(scenario_dir / "semantic_grid.csv")

    t = [float(r["t"]) for r in traj]
    px = [float(r["px"]) for r in traj]
    pz = [float(r["pz"]) for r in traj]
    cov = [float(r["cov_trace"]) for r in traj]

    ct = [float(r["t"]) for r in contact]
    left = [int(r["left"]) for r in contact]
    right = [int(r["right"]) for r in contact]

    xs = [int(r["x"]) for r in grid if float(r["confidence"]) > 0.15]
    ys = [int(r["y"]) for r in grid if float(r["confidence"]) > 0.15]
    occ = [float(r["occupancy"]) for r in grid if float(r["confidence"]) > 0.15]

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)
    axes[0].plot(px, pz)
    axes[0].set_title("Estimated base trajectory")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("z [m]")

    axes[1].step(ct, left, where="post", label="left")
    axes[1].step(ct, right, where="post", label="right")
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
