from __future__ import annotations

import csv
from pathlib import Path

import numpy as np



# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_csv_floats(text: str) -> list[float]:
    values = [t.strip() for t in text.split(",") if t.strip()]
    if not values:
        raise ValueError("No numeric values were provided.")
    return [float(v) for v in values]


def _parse_xyzwpr_arg(text: str) -> list[float]:
    """Parse a comma-separated XYZWPR string into 6 floats."""
    values = _parse_csv_floats(text)
    if len(values) != 6:
        raise ValueError(f"XYZWPR must have exactly 6 values, got {len(values)}.")
    return values


def _parse_column_range(spec: str, header: list[str]) -> list[int]:
    """
    Parse a column specification into a list of 0-based column indices.

    Accepts:
      - A numeric range  "5-11"   -> columns 5,6,7,8,9,10,11  (1-based, inclusive)
      - Comma-separated names     "J1,J2,J3,J4,J5,J6"
      - Mixed                     "5-11,13"
    """
    indices: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part and part.replace("-", "").replace(" ", "").isdigit():
            lo, hi = part.split("-", 1)
            indices.extend(range(int(lo) - 1, int(hi)))  # 1-based -> 0-based
        elif part.isdigit():
            indices.append(int(part) - 1)  # 1-based -> 0-based
        else:
            try:
                indices.append(header.index(part))
            except ValueError:
                raise ValueError(f"Column name '{part}' not found in CSV header.")
    return indices


def _matrix_to_xyz(matrix_flat: list[float]) -> tuple[float, float, float]:
    """Extract translation (x, y, z) from a flat row-major 4x4 transform matrix."""
    return matrix_flat[3], matrix_flat[7], matrix_flat[11]


def _matrix_to_axes(
    matrix_flat: list[float], scale: float = 50.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Return origin + X/Y/Z axis tip vectors for drawing a coordinate frame.
    `scale` controls arrow length in the same units as the translation (mm).
    """
    m = np.array(matrix_flat).reshape(4, 4)
    origin = m[:3, 3]
    x_tip = origin + m[:3, 0] * scale
    y_tip = origin + m[:3, 1] * scale
    z_tip = origin + m[:3, 2] * scale
    return origin, x_tip, y_tip, z_tip


# ---------------------------------------------------------------------------
# 3-D viewer
# ---------------------------------------------------------------------------

#def _launch_3d_viewer(
#    tool_matrices: list[list[float]],
#    frame_scale: float = 50.0,
#) -> None:
#    """
#    Open an interactive 3-D window using matplotlib with orbit (mouse) support.
#
#    Each tool pose is drawn as a small XYZ triad; the TCP path is drawn as a
#    continuous coloured line going from blue (first pose) to red (last pose).
#    """
#    import matplotlib.pyplot as plt
#    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
#    from mpl_toolkits.mplot3d.art3d import Line3DCollection
#    import matplotlib.cm as cm
#
#    positions = np.array([_matrix_to_xyz(m) for m in tool_matrices])  # (N, 3)
#    n = len(positions)
#
#    fig = plt.figure(figsize=(12, 8))
#    ax: Axes3D = fig.add_subplot(111, projection="3d")
#    ax.set_title("Robot Tool Path — drag to orbit", fontsize=13)
#
#    # --- Draw TCP path as a colour-mapped line (blue -> red over time) ------
#    colors = cm.coolwarm(np.linspace(0, 1, max(n - 1, 1)))
#    segments = [[positions[i], positions[i + 1]] for i in range(n - 1)]
#    if segments:
#        lc = Line3DCollection(segments, colors=colors, linewidths=1.5, zorder=1)
#        ax.add_collection3d(lc)
#
#    # --- Draw coordinate frames (thinned to at most ~200 triads) ------------
#    #step = max(1, n // 200)
#    #for i in range(0, n, step):
#    #    origin, x_tip, y_tip, z_tip = _matrix_to_axes(tool_matrices[i], scale=frame_scale)
#    #    ax.quiver(*origin, *(x_tip - origin), color="red",   length=1, normalize=False, linewidth=0.8)
#    #    ax.quiver(*origin, *(y_tip - origin), color="green", length=1, normalize=False, linewidth=0.8)
#    #    ax.quiver(*origin, *(z_tip - origin), color="blue",  length=1, normalize=False, linewidth=0.8)
#
#    # --- Start / end markers -------------------------------------------------
#    ax.scatter(*positions[0],  color="lime",    s=60, zorder=5, label="Start")
#    ax.scatter(*positions[-1], color="crimson", s=60, zorder=5, label="End")
#
#    # --- Axes labels & equal aspect ------------------------------------------
#    ax.set_xlabel("X (mm)")
#    ax.set_ylabel("Y (mm)")
#    ax.set_zlabel("Z (mm)")
#    ax.legend(loc="upper left", fontsize=9)
#
#    extents = np.array([
#        [positions[:, 0].min(), positions[:, 0].max()],
#        [positions[:, 1].min(), positions[:, 1].max()],
#        [positions[:, 2].min(), positions[:, 2].max()],
#    ])
#    max_range = (extents[:, 1] - extents[:, 0]).max() / 2
#    mid = extents.mean(axis=1)
#    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
#    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
#    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
#
#    plt.tight_layout()
#    plt.show()

# ---------------------------------------------------------------------------
# 3-D viewer
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# 3-D viewer
# ---------------------------------------------------------------------------

def _launch_3d_viewer(
    tool_matrices: list[list[float]],
    frame_scale: float = 50.0,
    register_poses: list[dict] | None = None,
) -> None:
    """
    Open an interactive 3-D viewer in the browser using Plotly/WebGL.

    - Log poses: small dots coloured blue->red over time.
    - Register poses: larger labelled dots, coloured by group.
    """
    import plotly.graph_objects as go

    positions = np.array([_matrix_to_xyz(m) for m in tool_matrices])  # (N, 3)
    n = len(positions)

    traces: list[go.BaseTraceType] = []

    # --- Log path as scalar-coloured dots -----------------------------------
    traces.append(go.Scatter3d(
        x=positions[:, 0],
        y=positions[:, 1],
        z=positions[:, 2],
        mode="markers",
        text=[f"Row {i}" for i in range(n)],
        hovertemplate="%{text}<br>X: %{x:.1f}<br>Y: %{y:.1f}<br>Z: %{z:.1f}<extra></extra>",
        name="Log poses",
        marker=dict(
            size=2,
            color=np.arange(n),
            colorscale="RdBu",
            reversescale=True,
            opacity=0.8,
            showscale=False,
        ),
    ))

    

    # --- Register poses ------------------------------------------------------
    if register_poses:
        palette = [
            "#FFD700", "#FF6B6B", "#4FC3F7", "#A5D6A7", "#CE93D8",
            "#FFAB40", "#80DEEA", "#EF9A9A", "#C5E1A5", "#B0BEC5",
        ]
        groups = list(dict.fromkeys(r["group"] for r in register_poses))
        group_color = {g: palette[i % len(palette)] for i, g in enumerate(groups)}

        # Group dots by group so legend has one entry per group
        from itertools import groupby
        sorted_regs = sorted(register_poses, key=lambda r: r["group"])
        for group, members in groupby(sorted_regs, key=lambda r: r["group"]):
            members = [m for m in members if m.get("_xyz") is not None]
            if not members:
                continue
            pts = np.array([m["_xyz"] for m in members])
            labels = [m["label"] for m in members]
            color = group_color[group]
            traces.append(go.Scatter3d(
                x=pts[:, 0],
                y=pts[:, 1],
                z=pts[:, 2],
                mode="markers+text",
                name=f"[{group}]",
                text=labels,
                textposition="top center",
                textfont=dict(color=color, size=11),
                marker=dict(size=7, color=color, line=dict(width=1, color="white")),
            ))

    fig = go.Figure(data=traces)
    fig.update_layout(
        title="Robot Tool Path",
        scene=dict(
            xaxis_title="X (mm)",
            yaxis_title="Y (mm)",
            zaxis_title="Z (mm)",
            bgcolor="#1e1e1e",
            xaxis=dict(backgroundcolor="#1e1e1e", gridcolor="#444", color="white"),
            yaxis=dict(backgroundcolor="#1e1e1e", gridcolor="#444", color="white"),
            zaxis=dict(backgroundcolor="#1e1e1e", gridcolor="#444", color="white"),
            aspectmode="data",
        ),
        paper_bgcolor="#1e1e1e",
        font=dict(color="white"),
        legend=dict(bgcolor="#2a2a2a", bordercolor="#555", borderwidth=1),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    #fig.show()
    import tempfile
    import webbrowser

    html_path = Path(tempfile.mktemp(suffix="_robot_tool_path.html"))
    fig.write_html(
        str(html_path),
        include_plotlyjs="cdn",
        post_script="""
        var btn = document.createElement('button');
        btn.innerText = '💾 Save as HTML';
        btn.style.cssText = 'position:fixed;top:10px;right:10px;z-index:9999;'
            + 'padding:8px 14px;background:#4FC3F7;color:#000;border:none;'
            + 'border-radius:6px;cursor:pointer;font-size:14px;font-weight:bold;';
        btn.onclick = function() {
            var blob = new Blob([document.documentElement.outerHTML],
                                {type: 'text/html'});
            var a = document.createElement('a');
            a.href = URL.createObjectURL(blob);
            a.download = 'robot_tool_path.html';
            a.click();
        };
        document.body.appendChild(btn);
        """
    )
    webbrowser.open(html_path.resolve().as_uri())



# ---------------------------------------------------------------------------
# Subparser registration (called from cli.py)
# ---------------------------------------------------------------------------

def add_view_joint_log_subparser(subparsers) -> None:
    """Register the view_joint_log subcommand onto an existing subparsers object."""
    from pathlib import Path
    vj = subparsers.add_parser(
        "view_joint_log",
        help="Replay a joint-value CSV log in a 3-D tool-path viewer",
    )
    vj.add_argument("log_file", help="Path to the CSV log file (first row must be a header).")
    vj.add_argument("robot_name", help="Robot kind string (e.g. lrmate200id7l).")
    vj.add_argument(
        "--robot-base",
        dest="robot_base",
        default=None,
        metavar="X,Y,Z,W,P,R",
        help="Robot base frame as XYZWPR (mm + degrees). Optional.",
    )
    vj.add_argument(
        "--tool-frame",
        dest="tool_frame",
        default=None,
        metavar="X,Y,Z,W,P,R",
        help="Tool frame as XYZWPR (mm + degrees). Optional.",
    )
    vj.add_argument(
        "--joint-columns",
        dest="joint_columns",
        default="1-6",
        metavar="RANGE_OR_NAMES",
        help='Column spec, e.g. "2-7" or "J1,J2,J3,J4,J5,J6". Default: "1-6".',
    )


    vj.add_argument(
        "--registers",
        dest="registers",
        default=None,
        metavar="REGISTERS_JSON",
        help="Optional JSON file with known joint register poses to overlay in the viewer.",
    )


def run_view_joint_log(args) -> int:
    """Alias so cli.py can import a consistently named symbol."""
    return run(args)


# ---------------------------------------------------------------------------
# Main entry point (called from cli.py)
# ---------------------------------------------------------------------------

def run(args) -> int:
    """
    Execute the view_joint_log command.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments from cli.py. Expected attributes:
            log_file, robot_name, joint_columns,
            robot_base, tool_frame,
            assembly_dir, platform_id, device_id
    """
    # Import here to avoid circular imports and defer heavy .NET loading
    from .api import FanucSimulator

    log_path = Path(args.log_file)
    if not log_path.exists():
        print(f"ERROR: Log file not found: {log_path}")
        return 1

    # --- Parse CSV -----------------------------------------------------------
    with log_path.open(newline="", encoding="utf-8-sig") as fh:
        reader = csv.reader(fh)
        raw_rows = list(reader)

    if not raw_rows:
        print("ERROR: Log file is empty.")
        return 1

    header = [h.strip() for h in raw_rows[0]]
    data_rows = raw_rows[1:]

    try:
        col_indices = _parse_column_range(args.joint_columns, header)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 1

    if len(col_indices) not in (6, 7):
        print(
            f"ERROR: joint_columns must resolve to 6 or 7 columns, "
            f"got {len(col_indices)}: {col_indices}"
        )
        return 1

    # --- Parse optional XYZWPR args -----------------------------------------
    robot_base: list[float] | None = None
    if args.robot_base:
        try:
            robot_base = _parse_xyzwpr_arg(args.robot_base)
        except ValueError as exc:
            print(f"ERROR: --robot-base: {exc}")
            return 1

    tool_frame: list[float] | None = None
    if args.tool_frame:
        try:
            tool_frame = _parse_xyzwpr_arg(args.tool_frame)
        except ValueError as exc:
            print(f"ERROR: --tool-frame: {exc}")
            return 1

    tool_id = 0 if tool_frame is not None else -1

    # --- Run FK for every row ------------------------------------------------
    assembly_dir = Path(args.assembly_dir)
    tool_matrices: list[list[float]] = []
    skipped = 0

    print(f"Running FK on {len(data_rows)} rows...")
    joint_rows: list[list[float]] = []
    row_numbers: list[int] = []
    for row_idx, row in enumerate(data_rows):
        try:
            joints = [float(row[i]) for i in col_indices]
        except (IndexError, ValueError):
            skipped += 1
            continue
        joint_rows.append(joints)
        row_numbers.append(row_idx + 2)

    if not joint_rows:
        print("ERROR: No valid data rows to run FK on.")
        return 1

    with FanucSimulator(
        assembly_dir=assembly_dir,
        robot_kind=args.robot_name,
        platform_id=args.platform_id,
        device_id=args.device_id,
    ) as sim:
        try:
            fk_batch = sim.fk_batch(
                joint_rows,
                robot_id=0,
                tool_id=tool_id,
                robot_base=robot_base,
            )
        except Exception as exc:
            print(f"ERROR: batched FK failed — {exc}")
            return 1

        link_tp = fk_batch.get("link_transforms") or []
        for i, joints in enumerate(joint_rows):
            line_no = row_numbers[i]
            if i >= len(link_tp):
                print(f"  WARNING row {line_no}: missing FK result")
                skipped += 1
                continue
            link_transforms = link_tp[i]
            if not link_transforms:
                skipped += 1
                continue
            tool_matrices.append(link_transforms[-1])

    if not tool_matrices:
        print("ERROR: No valid FK results were produced. Check column indices and data.")
        return 1

    print(f"  {len(tool_matrices)} poses computed ({skipped} rows skipped).")

    # --- Determine frame scale from data extents ----------------------------
    positions = np.array([_matrix_to_xyz(m) for m in tool_matrices])
    workspace_span = np.ptp(positions, axis=0).max()
    frame_scale = max(10.0, workspace_span * 0.03)

    # --- Load and resolve register poses ------------------------------------
    register_poses: list[dict] | None = None
    registers_path = getattr(args, "registers", None)
    if registers_path:
        import json
        reg_path = Path(registers_path)
        if not reg_path.exists():
            print(f"ERROR: Registers file not found: {reg_path}")
            return 1
        with reg_path.open(encoding="utf-8") as fh:
            reg_data = json.load(fh)
        raw_registers = reg_data.get("registers", [])
        register_poses = []
        print(f"Resolving FK for {len(raw_registers)} register poses...")
        reg_joint_rows: list[list[float]] = []
        valid_regs: list[dict] = []
        for reg in raw_registers:
            joints = reg.get("joints")
            if not isinstance(joints, list):
                print(f"  WARNING register '{reg.get('label', '?')}': missing joints list.")
                continue
            try:
                jt = [float(v) for v in joints]
            except (TypeError, ValueError):
                print(f"  WARNING register '{reg.get('label', '?')}': invalid joints.")
                continue
            reg_joint_rows.append(jt)
            valid_regs.append(reg)

        with FanucSimulator(
            assembly_dir=assembly_dir,
            robot_kind=args.robot_name,
            platform_id=args.platform_id,
            device_id=args.device_id,
        ) as sim:
            if reg_joint_rows:
                try:
                    reg_fk = sim.fk_batch(
                        reg_joint_rows,
                        robot_id=0,
                        tool_id=tool_id,
                        robot_base=robot_base,
                    )
                except Exception as exc:
                    print(f"ERROR: batched FK (registers) failed — {exc}")
                    return 1
                reg_lt = reg_fk.get("link_transforms") or []
                for i, reg in enumerate(valid_regs):
                    if i >= len(reg_lt):
                        print(f"  WARNING register '{reg.get('label', '?')}': missing FK result")
                        continue
                    lt = reg_lt[i]
                    if lt:
                        reg["_xyz"] = _matrix_to_xyz(lt[-1])
                        register_poses.append(reg)

    print("Launching 3-D viewer — drag to orbit, scroll to zoom, right-drag to pan.")
    _launch_3d_viewer(tool_matrices, frame_scale=frame_scale, register_poses=register_poses)
    return 0


# ---------------------------------------------------------------------------
# Standalone entry point  (uv run scripts/view_joint_log.py ...)
# ---------------------------------------------------------------------------

def _build_standalone_parser() -> "argparse.ArgumentParser":
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(
        prog="view_joint_log",
        description="Replay a joint-value CSV log in a 3-D tool-path viewer",
    )
    parser.add_argument("log_file", help="Path to the CSV log file.")
    parser.add_argument("robot_name", help="Robot kind (e.g. lrmate200id7l).")
    parser.add_argument(
        "--assembly-dir",
        dest="assembly_dir",
        type=Path,
        default=Path("dotnet_bridge/bin/Debug/net8.0"),
        help="Folder containing the .NET assemblies.",
    )
    parser.add_argument(
        "--robot-base",
        dest="robot_base",
        default=None,
        metavar="X,Y,Z,W,P,R",
        help="Base frame as XYZWPR (mm + degrees). Optional.",
    )
    parser.add_argument(
        "--tool-frame",
        dest="tool_frame",
        default=None,
        metavar="X,Y,Z,W,P,R",
        help="Tool frame as XYZWPR (mm + degrees). Optional.",
    )
    parser.add_argument(
        "--joint-columns",
        dest="joint_columns",
        default="1-6",
        metavar="RANGE_OR_NAMES",
        help='Column spec, e.g. "2-7" or "J1,J2,J3,J4,J5,J6". Default: "1-6".',
    )
    parser.add_argument("--platform-id", dest="platform_id", type=int, default=0)
    parser.add_argument("--device-id", dest="device_id", type=int, default=0)
    parser.add_argument(
        "--registers",
        dest="registers",
        default=None,
        metavar="REGISTERS_JSON",
        help="Optional JSON file with known joint register poses to overlay in the viewer.",
    )
    return parser


if __name__ == "__main__":
    import sys
    import argparse
    from pathlib import Path

    # Make the package importable when run as a script directly
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from gb_robotics_py.api import FanucSimulator  # noqa: E402 (re-import as absolute)

    parser = _build_standalone_parser()
    args = parser.parse_args()
    raise SystemExit(run(args))