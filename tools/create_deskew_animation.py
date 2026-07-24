#!/usr/bin/env python3
"""Render individual plots for one frame's point-cloud deskew iterations."""

import argparse
from pathlib import Path
from typing import Tuple

import matplotlib

import numpy as np


# Keep the on-screen figure size compact while preserving detail in saved plots.
SAVE_DPI = 200


def read_pcd(path: Path) -> np.ndarray:
    """Read binary PCD files written by PCL, preserving x/y/z/intensity."""
    with path.open("rb") as stream:
        header = {}
        while True:
            line = stream.readline().decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key] = values
            if key == "DATA":
                break
        if header["DATA"][0] != "binary":
            raise ValueError(f"Only binary PCD is supported: {path}")

        fields = header["FIELDS"]
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header["COUNT"]]
        points = int(header["POINTS"][0])

        type_map = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 2): "<u2", ("U", 4): "<u4", ("I", 2): "<i2", ("I", 4): "<i4"}
        dtype_fields = []
        offset = 0
        for field, size, field_type, count in zip(fields, sizes, types, counts):
            base_type = type_map.get((field_type, size))
            if base_type is None or count != 1:
                raise ValueError(f"Unsupported PCD field in {path}: {field} {field_type}{size} x {count}")
            dtype_fields.append((field, base_type))
            offset += size
        dtype = np.dtype(dtype_fields, align=False)
        if dtype.itemsize != offset:
            raise ValueError(f"Unexpected PCD layout: {path}")
        cloud = np.fromfile(stream, dtype=dtype, count=points)

    required = {"x", "y", "z"}
    if not required.issubset(cloud.dtype.names):
        raise ValueError(f"PCD misses xyz fields: {path}")
    intensity = cloud["intensity"] if "intensity" in cloud.dtype.names else np.zeros(points, dtype=np.float32)
    return np.column_stack((cloud["x"], cloud["y"], cloud["z"], intensity)).astype(np.float64)


def valid_points(points: np.ndarray, max_range: float = 120.0) -> np.ndarray:
    xyz = points[:, :3]
    finite = np.isfinite(xyz).all(axis=1) & np.isfinite(points[:, 3])
    range_ok = np.linalg.norm(xyz, axis=1) > 0.1
    range_ok &= np.linalg.norm(xyz, axis=1) < max_range
    return points[finite & range_ok]


def evenly_subsample(points: np.ndarray, max_points: int) -> np.ndarray:
    if len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def finish_figure(fig, path: Path, show: bool) -> None:
    """Save the plot and optionally present it before advancing to the next one."""
    fig.savefig(path, facecolor=fig.get_facecolor(), dpi=SAVE_DPI)
    if show:
        fig.canvas.manager.set_window_title(path.stem)
        # Block here so one optimizer iteration is inspected at a time.
        plt.show(block=True)
    plt.close(fig)


def make_axes(bounds: Tuple[np.ndarray, np.ndarray]):
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor="#10151b")
    ax = fig.add_subplot(111, projection="3d", facecolor="#10151b")
    low, high = bounds
    ax.set_xlim(low[0], high[0])
    ax.set_ylim(low[1], high[1])
    ax.set_zlim(low[2], high[2])
    ax.view_init(elev=24, azim=-128)
    ax.set_box_aspect(high - low)
    # Keep the scene itself, but remove every coordinate-system element.
    ax.set_axis_off()
    return fig, ax


def save_cloud_frame(path: Path, cloud: np.ndarray, bounds, show: bool) -> None:
    fig, ax = make_axes(bounds)
    cloud = valid_points(cloud)
    low, high = bounds
    in_view = np.all((cloud[:, :3] >= low) & (cloud[:, :3] <= high), axis=1)
    cloud = evenly_subsample(cloud[in_view], 90000)
    intensity = cloud[:, 3]
    lo, hi = np.percentile(intensity, [2, 98]) if len(intensity) else (0.0, 1.0)
    if hi <= lo:
        hi = lo + 1.0
    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], c=intensity, cmap="turbo",
               vmin=lo, vmax=hi, s=1.1, alpha=0.82, linewidths=0, depthshade=False)
    finish_figure(fig, path, show)


def save_feature_frame(path: Path, prev: np.ndarray, current: np.ndarray, bounds, show: bool) -> None:
    # A 2D top-down plot makes one optimizer iteration directly comparable to
    # the next one. Only the fixed reference and this iteration's current
    # features are drawn; no previous iteration is overlaid.
    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100, facecolor="#10151b")
    ax.set_facecolor("#10151b")
    low, high = bounds
    ax.set_xlim(low[0], high[0])
    ax.set_ylim(low[1], high[1])
    ax.set_aspect("equal", adjustable="box")
    ax.set_axis_off()
    prev = valid_points(prev)
    current = valid_points(current)
    count = min(len(prev), len(current))
    prev = prev[:count]
    current = current[:count]

    # Lines make the residual of the current iteration visible in the XY plane.
    for p, q in zip(prev, current):
        ax.plot((p[0], q[0]), (p[1], q[1]), color="#aeb7c2", alpha=0.50, linewidth=0.9, zorder=1)
    ax.scatter(prev[:, 0], prev[:, 1], c="#35a7ff", s=42, linewidths=0.55,
               edgecolors="#dceaff", zorder=3, label="Previous-frame fixed reference")
    ax.scatter(current[:, 0], current[:, 1], c="#ff5d5d", s=42, linewidths=0.55,
               edgecolors="#ffe0e0", zorder=4, label="Current-frame features")
    legend = ax.legend(loc="upper right", frameon=True, facecolor="#202833", edgecolor="#607080", fontsize=9)
    for text in legend.get_texts():
        text.set_color("white")
    finish_figure(fig, path, show)


def compute_bounds(*clouds: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    merged = np.concatenate([valid_points(cloud)[:, :3] for cloud in clouds if len(cloud)], axis=0)
    low = np.percentile(merged, 1, axis=0)
    high = np.percentile(merged, 99, axis=0)
    margin = np.maximum((high - low) * 0.08, 1.0)
    return low - margin, high + margin


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("lidar_output"))
    parser.add_argument("--frame", type=int, default=228)
    parser.add_argument("--show", action="store_true",
                        help="Open one interactive Figure at a time; close it to advance.")
    args = parser.parse_args()

    global plt
    # Qt is available in this ROS desktop environment. TkAgg depends on
    # Pillow's ImageTk module, which is not consistently installed here.
    matplotlib.use("Qt5Agg" if args.show else "Agg")
    import matplotlib.pyplot as plt

    output_dir = args.output_dir.resolve()
    frame = args.frame
    raw = read_pcd(output_dir / "pcd_raw" / f"raw_frame_{frame}.pcd")
    deskewed = read_pcd(output_dir / "pcd_deskewed" / f"deskewed_frame_{frame}.pcd")
    prev = read_pcd(output_dir / "pcd_features_raw" / f"frame_{frame}_prev_features_raw.pcd")
    current_raw = read_pcd(output_dir / "pcd_features_raw" / f"frame_{frame}_cur_features_raw.pcd")
    iteration_paths = sorted((output_dir / "pcd_joint_iterations").glob(f"frame_{frame}_iter_*_cur_in_prev_reference.pcd"),
                             key=lambda item: int(item.stem.split("_iter_")[1].split("_")[0]))
    if not iteration_paths:
        raise FileNotFoundError(f"No joint-optimization PCD files found for frame {frame}")
    iterations = [read_pcd(path) for path in iteration_paths]

    # Feature correspondence and optimization frames use this shared local
    # field of view.  Raw/final point clouds are rendered in the same bounds,
    # rather than zooming out to the entire scan and making features tiny.
    bounds = compute_bounds(prev, current_raw, *iterations)
    render_dir = output_dir / "deskew_animation" / f"frame_{frame}"
    render_dir.mkdir(parents=True, exist_ok=True)
    plot_dir = render_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    images = []
    image_path = plot_dir / "00_raw_cloud.png"
    save_cloud_frame(image_path, raw, bounds, args.show)
    images.append(image_path)

    image_path = plot_dir / "01_raw_feature_pairs.png"
    save_feature_frame(image_path, prev, current_raw, bounds, args.show)
    images.append(image_path)

    for index, cloud in enumerate(iterations):
        image_path = plot_dir / f"{index + 2:02d}_iteration_{index:02d}.png"
        save_feature_frame(image_path, prev, cloud, bounds, args.show)
        images.append(image_path)

    image_path = plot_dir / f"{len(images):02d}_final_deskewed_cloud.png"
    save_cloud_frame(image_path, deskewed, bounds, args.show)
    images.append(image_path)

    print(f"Wrote {len(images)} individual plots: {plot_dir}")


if __name__ == "__main__":
    main()
