
# cli.py
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .api import FanucSimulator
from .view_joint_log import run_view_joint_log,add_view_joint_log_subparser



# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_csv_floats(text: str) -> list[float]:
    values = [t.strip() for t in text.split(",") if t.strip()]
    if not values:
        raise ValueError("No numeric values were provided.")
    return [float(v) for v in values]


def _parse_fk_joints(text: str) -> list[float]:
    joints = _parse_csv_floats(text)
    if len(joints) not in (6, 7):
        raise ValueError("FK joints must contain 6 or 7 comma-separated values.")
    return joints


def _parse_ik_matrix(text: str) -> list[float]:
    matrix = _parse_csv_floats(text)
    if len(matrix) != 16:
        raise ValueError("IK matrix must contain exactly 16 comma-separated values.")
    return matrix


# ---------------------------------------------------------------------------
# Argument-parser construction
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="gb-robotics",
        description="CLI for testing GB Robotics FK/IK bridge",
    )
    parser.add_argument(
        "--assembly-dir",
        type=Path,
        default=Path("dotnet_bridge/bin/Debug/net8.0"),
        help="Folder containing GBRobotics.PythonBridge.dll and dependencies",
    )
    parser.add_argument(
        "--robot-kind",
        default="lrmate200id7l",
        help="Robot kind (e.g. lrmate200id7l, lr10ia10)",
    )
    parser.add_argument("--platform-id", type=int, default=0, help="OpenCL platform id")
    parser.add_argument("--device-id", type=int, default=0, help="OpenCL device id")

    subparsers = parser.add_subparsers(dest="command", required=True)

    # --- fk ------------------------------------------------------------------
    fk_parser = subparsers.add_parser("fk", help="Run forward kinematics")
    fk_parser.add_argument(
        "--joints",
        required=True,
        help="Comma-separated joints (6 or 7 values). Example: 0,0.1,0.2,0,0,0",
    )
    fk_parser.add_argument("--robot-id", type=int, default=0)
    fk_parser.add_argument("--tool-id", type=int, default=-1)

    # --- ik ------------------------------------------------------------------
    ik_parser = subparsers.add_parser("ik", help="Run inverse kinematics")
    ik_parser.add_argument(
        "--matrix",
        required=True,
        help="Flattened 4x4 transform matrix (16 comma-separated values, row-major)",
    )
    ik_parser.add_argument("--robot-id", type=int, default=0)

    # --- view_joint_log (registered from its own module) ---------------------
    add_view_joint_log_subparser(subparsers)

    return parser


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "view_joint_log":
        return run_view_joint_log(args)

    try:
        with FanucSimulator(
            assembly_dir=args.assembly_dir,
            robot_kind=args.robot_kind,
            platform_id=args.platform_id,
            device_id=args.device_id,
        ) as sim:
            if args.command == "fk":
                joints = _parse_fk_joints(args.joints)
                result = sim.fk(joints, robot_id=args.robot_id, tool_id=args.tool_id)
            else:
                matrix = _parse_ik_matrix(args.matrix)
                result = sim.ik([matrix], robot_id=args.robot_id)

        print(json.dumps(result, indent=2))
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())