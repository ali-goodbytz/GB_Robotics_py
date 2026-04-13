"""CLI: visualize a motion graph in 3-D TCP space."""

from __future__ import annotations

import json
import os
import sys
import webbrowser
from argparse import Namespace
from pathlib import Path
from tempfile import NamedTemporaryFile

from robot_profiler.export.json_exporter import load_graph_json

from .vis_graph_data import (
    build_scene,
    matrix_row_major_to_xyz,
    parse_xyzwpr,
)
from .vis_graph_html import write_vis_graph_html


def _try_inject_gb_python_path(
    cli_root: Path | None,
    env_root: str | None,
    metadata: dict[str, object],
) -> str | None:
    """
    Prepend the GB_Robotics_py repo root to sys.path so ``gb_robotics_py`` imports.

    Returns an error message if a path was given but is invalid; otherwise None.
    """
    raw: Path | None = cli_root
    if raw is None and env_root and env_root.strip():
        raw = Path(env_root.strip())
    if raw is None:
        meta_val = metadata.get("gb_python_root")
        if isinstance(meta_val, str) and meta_val.strip():
            raw = Path(meta_val.strip())
    if raw is None:
        return None

    root = raw.expanduser().resolve()
    if not root.exists():
        return f"GB Python root does not exist: {root}"

    insert: Path | None = None
    pkg = root / "gb_robotics_py"
    if pkg.is_dir() and (pkg / "__init__.py").is_file():
        insert = root
    elif root.name == "gb_robotics_py" and (root / "__init__.py").is_file():
        insert = root.parent
    if insert is None:
        return (
            f"Not a GB_Robotics_py repo root (missing gb_robotics_py package): {root}"
        )

    prefix = str(insert)
    if prefix not in sys.path:
        sys.path.insert(0, prefix)
    return None


def add_view_graph_subparser(subparsers) -> None:
    """Register ``view_graph`` on the main ``gb-robotics`` CLI."""
    parser = subparsers.add_parser(
        "view_graph",
        help="Open a 3-D browser view of a motion graph (TCP from FK on samples)",
    )
    parser.add_argument(
        "graph_json",
        type=Path,
        help="Path to motion graph JSON (with joints and edge samples)",
    )
    parser.add_argument(
        "--registers",
        type=Path,
        default=None,
        metavar="PATH",
        help=(
            "Optional registers JSON (same shape as view_joint_log: "
            "{registers: [...]})"
        ),
    )
    parser.add_argument(
        "--tool-frame",
        dest="tool_frame",
        default=None,
        metavar="X,Y,Z,W,P,R",
        help=(
            "Tool frame as XYZWPR (mm, degrees), same convention as view_joint_log. "
            "If omitted, FK uses tool_id=-1 (no tool offset)."
        ),
    )
    parser.add_argument(
        "--robot-base",
        dest="robot_base",
        default=None,
        metavar="X,Y,Z,W,P,R",
        help="Optional robot base as XYZWPR.",
    )
    parser.add_argument(
        "--robot-mode",
        dest="robot_mode",
        action="store_true",
        help=(
            "Use the main CLI --robot-kind and --assembly-dir (place them before "
            "'view_graph'). Otherwise prefer graph.metadata, then fall back to those "
            "defaults."
        ),
    )
    parser.add_argument(
        "--gb-python-root",
        dest="gb_python_root",
        type=Path,
        default=None,
        metavar="DIR",
        help=(
            "Folder that contains the gb_robotics_py package (GB_Robotics_py repo "
            "root). Prepends sys.path if gb_robotics_py is not installed. "
            "Or set GB_ROBOTICS_PY_ROOT or graph.metadata['gb_python_root']."
        ),
    )
    parser.add_argument("--platform-id", type=int, default=0)
    parser.add_argument("--device-id", type=int, default=0)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Write HTML to this path (default: temp file and open browser)",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not open a browser (only useful with --output)",
    )
    parser.add_argument(
        "--node-color-by",
        dest="node_color_by",
        choices=("source", "group", "type"),
        default="source",
        help=(
            "Color graph nodes categorically by robot_profiler Node field: "
            "'source', 'group', or 'type' (node_type). In the HTML viewer you can "
            "switch mode from the toolbar."
        ),
    )


def run_view_graph(args: Namespace) -> int:
    """Entry point for ``gb-robotics view_graph``."""
    return run(args)


def run(args: Namespace) -> int:
    graph_path = args.graph_json.resolve()
    if not graph_path.is_file():
        print(f"ERROR: Graph file not found: {graph_path}")
        return 1

    _tool_frame: list[float] | None = None
    if args.tool_frame:
        try:
            _tool_frame = parse_xyzwpr(args.tool_frame)
        except ValueError as exc:
            print(f"ERROR: --tool-frame: {exc}")
            return 1

    robot_base = None
    if args.robot_base:
        try:
            robot_base = parse_xyzwpr(args.robot_base)
        except ValueError as exc:
            print(f"ERROR: --robot-base: {exc}")
            return 1

    graph = load_graph_json(graph_path)
    meta = graph.metadata

    if getattr(args, "robot_mode", False):
        robot_kind = getattr(args, "robot_kind", None)
        assembly_dir = getattr(args, "assembly_dir", None)
    else:
        robot_kind = meta.get("robot_kind") or getattr(args, "robot_kind", None)
        raw_ad = meta.get("assembly_dir")
        assembly_dir = Path(raw_ad) if raw_ad else getattr(args, "assembly_dir", None)
    if isinstance(assembly_dir, str):
        assembly_dir = Path(assembly_dir)

    if not robot_kind or not assembly_dir:
        print(
            "ERROR: Forward kinematics needs robot_kind and assembly_dir. "
            "Use --robot-mode with main CLI --robot-kind / --assembly-dir before "
            "'view_graph', or set robot_kind and assembly_dir in graph JSON metadata."
        )
        return 1

    registers_raw: list[dict] | None = None
    if args.registers is not None:
        reg_path = args.registers.resolve()
        if not reg_path.is_file():
            print(f"ERROR: Registers file not found: {reg_path}")
            return 1
        reg_data = json.loads(reg_path.read_text(encoding="utf-8"))
        raw = reg_data.get("registers")
        if not isinstance(raw, list):
            print("ERROR: registers JSON must contain a 'registers' array.")
            return 1
        registers_raw = [dict(x) for x in raw]

    inj_err = _try_inject_gb_python_path(
        getattr(args, "gb_python_root", None),
        os.environ.get("GB_ROBOTICS_PY_ROOT"),
        graph.metadata,
    )
    if inj_err:
        print(f"ERROR: {inj_err}")
        return 1

    try:
        from gb_robotics_py.api import FanucSimulator
    except ImportError:
        print(
            "ERROR: gb_robotics_py is not importable. Use --gb-python-root DIR "
            "(repo root containing gb_robotics_py), set GB_ROBOTICS_PY_ROOT, install "
            "the package, or extend PYTHONPATH (see GB_Robotics_py repo)."
        )
        return 1

    # Like view_joint_log: tool frame present => tool_id 0 on the bridge.
    tool_id = 0 if _tool_frame is not None else -1

    # Pass None (not [None]) when no tool frame — pythonnet cannot marshal [None] to double[][].
    tool_frames_kw = (
        [list(map(float, _tool_frame))] if _tool_frame is not None else None
    )

    with FanucSimulator(
        assembly_dir=assembly_dir,
        robot_kind=str(robot_kind),
        user_frames=None,
        tool_frames=tool_frames_kw,
        platform_id=int(args.platform_id),
        device_id=int(args.device_id),
    ) as sim:
        cache: dict[tuple[float, ...], tuple[float, float, float] | None] = {}

        def joint_to_tcp_one(
            joints: tuple[float, ...] | list[float],
        ) -> tuple[float, float, float] | None:
            key = tuple(float(x) for x in joints)
            if key in cache:
                return cache[key]
            try:
                fk = sim.fk(
                    list(key),
                    robot_id=0,
                    tool_id=tool_id,
                    robot_base=robot_base,
                )
                lt = fk.get("link_transforms") or []
                if not lt:
                    cache[key] = None
                else:
                    cache[key] = matrix_row_major_to_xyz(lt[-1])
            except Exception:
                cache[key] = None
            return cache[key]

        nodes, edges, registers, warnings = build_scene(
            graph,
            joint_to_tcp_one,
            registers_raw=registers_raw,
        )

    for w in warnings:
        print(f"WARNING: {w}")

    if not nodes:
        print("ERROR: No nodes with valid FK; nothing to draw.")
        return 1

    out_path: Path
    if args.output is not None:
        out_path = args.output.resolve()
    else:
        tmp = NamedTemporaryFile(
            delete=False,
            suffix="_vis_graph.html",
            prefix="robot_profiler_",
        )
        tmp.close()
        out_path = Path(tmp.name)

    try:
        write_vis_graph_html(
            path=out_path,
            nodes=nodes,
            edges=edges,
            registers=registers,
            graph_title=f"Motion graph TCP — {graph_path.name}",
            node_color_by=getattr(args, "node_color_by", "source") or "source",
        )
    except ImportError as exc:
        print(f"ERROR: {exc}")
        return 1

    print(f"Wrote {out_path}")
    if not args.no_browser:
        webbrowser.open(out_path.resolve().as_uri())
    return 0
