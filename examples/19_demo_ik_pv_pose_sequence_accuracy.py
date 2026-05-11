"""20_demo_ik_pv_pose_sequence_accuracy.py - multi-pose IK + PV accuracy demo.

Workflow:
  1. Edit the target pose list in this file.
  2. For each target pose:
     - go home
     - execute IK + PV motion
     - read back actual pose
     - report XYZ axis error and absolute positioning error
  3. After all poses finish:
     - report mean absolute XYZ error
     - report mean absolute positioning error
     - visualize per-pose errors with matplotlib
"""

import argparse
import time

import numpy as np

import alicia_m_sdk
from _common import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array


# ===== User config: edit target poses here =====
# Pose format: [x, y, z, qx, qy, qz, qw]
TEST_TARGET_POSES = [
    [0.29,  0.14, 0.49, 0.0, 0.69, 0.0, 0.72],
    [0.31,  0.11, 0.49, 0.0, 0.74, 0.0, 0.68],
    [0.30, 0.11, 0.37, 0.0, 0.74, 0.0, 0.67],
    [0.32,  -0.07, 0.39, 0.0,0.74, 0.0,    0.68],
    [0.50, -0.14, 0.34, 0.0,    0.85, 0.0,    0.53],
    [0.33, - 0.21, 0.33, 0.0,    0.85, 0.0,    0.53],
    [0.43, -0.16, 0.34, 0.0,   0.7, 0.0,  0.72],
    [0.38,  -0.12, 0.29, -0.0, 0.7, 0.0, 0.72],
]
TEST_SPEED = 8.0
TEST_SETTLE_TIME = 0.8
SHOW_PLOT = True


def _to_bool(value):
    if isinstance(value, list):
        return bool(value[0]) if value else False
    return bool(value)


def _plot_pose_sequence_results(results, mean_axis_abs_error_mm, mean_abs_error_mm):
    """Plot per-pose XYZ axis errors and absolute positioning errors."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        beauty_print("matplotlib is not available, skipping visualization.", type="warning")
        return

    if not results:
        beauty_print("No successful poses to visualize.", type="warning")
        return

    pose_ids = np.arange(1, len(results) + 1)
    axis_errors_mm = np.asarray([item["axis_error_mm"] for item in results], dtype=float)
    abs_errors_mm = np.asarray([item["abs_error_mm"] for item in results], dtype=float)

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    width = 0.22
    offsets = (-width, 0.0, width)
    labels = ("dx", "dy", "dz")
    colors = ("C0", "C1", "C2")
    for idx, (offset, label, color) in enumerate(zip(offsets, labels, colors)):
        axes[0].bar(
            pose_ids + offset,
            axis_errors_mm[:, idx],
            width=width,
            color=color,
            alpha=0.85,
            label=f"{label} (mm)",
        )

    axes[0].axhline(0.0, color="0.35", linewidth=1.0)
    axes[0].set_title("XYZ axis error for each target pose")
    axes[0].set_ylabel("Axis error (mm)")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[0].legend(loc="best", fontsize=9)
    axes[0].text(
        0.98,
        0.95,
        (
            f"Mean |dx| = {mean_axis_abs_error_mm[0]:.3f} mm\n"
            f"Mean |dy| = {mean_axis_abs_error_mm[1]:.3f} mm\n"
            f"Mean |dz| = {mean_axis_abs_error_mm[2]:.3f} mm"
        ),
        transform=axes[0].transAxes,
        ha="right",
        va="top",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9, "edgecolor": "0.7"},
    )

    axes[1].bar(pose_ids, abs_errors_mm, color="C3", alpha=0.85, label="Absolute positioning error")
    axes[1].axhline(
        mean_abs_error_mm,
        color="C4",
        linestyle="--",
        linewidth=1.2,
        label=f"Mean absolute error = {mean_abs_error_mm:.3f} mm",
    )
    axes[1].set_title("Absolute positioning error for each target pose")
    axes[1].set_xlabel("Pose index")
    axes[1].set_ylabel("Absolute error (mm)")
    axes[1].set_xticks(pose_ids)
    axes[1].grid(True, axis="y", alpha=0.3)
    axes[1].legend(loc="best", fontsize=9)

    plt.tight_layout()
    plt.show()


def main():
    beauty_print("Demo: multi-pose IK + PV accuracy", type="module")

    parser = argparse.ArgumentParser(
        description="Run Alicia-M multi-pose IK + PV accuracy test.",
    )
    add_port_argument(parser)
    parser.add_argument("--speed", type=float, default=TEST_SPEED, help="PV motion speed.")
    parser.add_argument(
        "--settle-time",
        type=float,
        default=TEST_SETTLE_TIME,
        help="Extra settling time after motion completion, in seconds.",
    )
    parser.add_argument(
        "--ik-method",
        type=str,
        default="dls",
        help="IK method passed to robot.set_pose(), for example dls.",
    )
    parser.add_argument("--max-iters", type=int, default=500, help="Maximum IK iterations.")
    parser.add_argument("--pos-tol", type=float, default=1e-3, help="IK position tolerance in meters.")
    parser.add_argument("--ori-tol", type=float, default=1e-2, help="IK orientation tolerance in radians.")
    parser.add_argument(
        "--num-initial-guesses",
        type=int,
        default=10,
        help="Number of IK initial guesses.",
    )
    parser.add_argument(
        "--plot",
        action=argparse.BooleanOptionalAction,
        default=SHOW_PLOT,
        help="Show matplotlib visualization after the test.",
    )
    args = parser.parse_args()

    if not TEST_TARGET_POSES:
        raise ValueError("TEST_TARGET_POSES is empty.")

    beauty_print(f"Pose count: {len(TEST_TARGET_POSES)}", type="info")
    for idx, pose in enumerate(TEST_TARGET_POSES, start=1):
        beauty_print(
            f"Pose {idx}: {beauty_print_array(pose, precision=5)}",
            type="info",
        )

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print(
        f"Robot connected (current mode: {robot.control_mode.value.upper()})",
        type="success",
    )

    if robot.control_mode.value != "pv":
        beauty_print(
            "This demo requires PV mode. Switching mode will briefly disable the arm.",
            type="warning",
        )
        input("Press Enter to switch to PV mode...")
        robot.switch_mode("pv")
        beauty_print("Switched to PV mode.", type="success")

    successful_results = []
    failed_poses = []

    try:
        for pose_idx, target_pose in enumerate(TEST_TARGET_POSES, start=1):
            target_position = np.asarray(target_pose[:3], dtype=float)

            beauty_print(f"Pose {pose_idx}/{len(TEST_TARGET_POSES)}", type="module")
            beauty_print(
                f"Target position (m): {beauty_print_array(target_position, precision=5)}",
                type="info",
            )
            beauty_print(
                f"Target quaternion (xyzw): {beauty_print_array(target_pose[3:], precision=5)}",
                type="info",
            )

            beauty_print("Returning to home...", type="info")
            robot.go_home(speed=args.speed)
            time.sleep(args.settle_time)

            beauty_print("Executing IK target in PV mode...", type="info")
            ik_result = robot.set_pose(
                target_pose=target_pose,
                method=args.ik_method,
                execute=True,
                speed=args.speed,
                max_iters=args.max_iters,
                pos_tol=args.pos_tol,
                ori_tol=args.ori_tol,
                num_initial_guesses=args.num_initial_guesses,
                use_analytic_jacobian=True,
            )

            ik_success = _to_bool(ik_result.get("success", False))
            motion_executed = _to_bool(ik_result.get("motion_executed", False))
            if not ik_success or not motion_executed:
                failed_poses.append((pose_idx, "IK solve or motion execution failed"))
                beauty_print(
                    f"Pose {pose_idx} failed: IK success={ik_success}, motion_executed={motion_executed}",
                    type="warning",
                )
                continue

            time.sleep(args.settle_time)
            pose_info = robot.get_pose()
            if pose_info is None:
                failed_poses.append((pose_idx, "Failed to read actual pose"))
                beauty_print(f"Pose {pose_idx} failed: cannot read actual pose.", type="warning")
                continue

            actual_position = np.asarray(pose_info["position"], dtype=float)
            axis_error_mm = (actual_position - target_position) * 1000.0
            axis_abs_error_mm = np.abs(axis_error_mm)
            abs_error_mm = float(np.linalg.norm(actual_position - target_position) * 1000.0)

            successful_results.append(
                {
                    "pose_idx": pose_idx,
                    "target_pose": np.asarray(target_pose, dtype=float),
                    "target_position": target_position,
                    "actual_position": actual_position,
                    "axis_error_mm": axis_error_mm,
                    "axis_abs_error_mm": axis_abs_error_mm,
                    "abs_error_mm": abs_error_mm,
                }
            )

            beauty_print(
                f"Actual position (m): {beauty_print_array(actual_position, precision=5)}",
                type="info",
            )
            beauty_print(
                f"XYZ axis error (mm): {beauty_print_array(axis_error_mm, precision=3)}",
                type="info",
            )
            beauty_print(
                f"Absolute positioning error: {abs_error_mm:.3f} mm",
                type="success",
            )

        beauty_print("Summary", type="module")
        beauty_print(
            f"Successful poses: {len(successful_results)}/{len(TEST_TARGET_POSES)}",
            type="info",
        )
        beauty_print(f"Failed poses: {len(failed_poses)}", type="info")
        for pose_idx, reason in failed_poses:
            beauty_print(f"  Pose {pose_idx}: {reason}", type="warning")

        if successful_results:
            axis_abs_error_mm = np.asarray(
                [item["axis_abs_error_mm"] for item in successful_results],
                dtype=float,
            )
            abs_errors_mm = np.asarray(
                [item["abs_error_mm"] for item in successful_results],
                dtype=float,
            )
            mean_axis_abs_error_mm = np.mean(axis_abs_error_mm, axis=0)
            mean_abs_error_mm = float(np.mean(abs_errors_mm))

            beauty_print(
                f"Mean absolute X error: {mean_axis_abs_error_mm[0]:.3f} mm",
                type="info",
            )
            beauty_print(
                f"Mean absolute Y error: {mean_axis_abs_error_mm[1]:.3f} mm",
                type="info",
            )
            beauty_print(
                f"Mean absolute Z error: {mean_axis_abs_error_mm[2]:.3f} mm",
                type="info",
            )
            beauty_print(
                f"Mean absolute positioning error: {mean_abs_error_mm:.3f} mm",
                type="success",
            )

            if args.plot:
                beauty_print("Showing matplotlib visualization...", type="info")
                _plot_pose_sequence_results(
                    successful_results,
                    mean_axis_abs_error_mm,
                    mean_abs_error_mm,
                )
        else:
            beauty_print("No successful poses, statistics unavailable.", type="warning")

    except KeyboardInterrupt:
        beauty_print("\nUser interrupted.", type="warning")
    finally:
        robot.disconnect()
        beauty_print("Disconnected.", type="info")


if __name__ == "__main__":
    main()
