# GB Robotics Python API

This repository provides a Python API for FK/IK calls and 3-D log review using your `GB_Robotics` .NET assemblies via `pythonnet`.

---

## Important about `.dll` on Linux

`GB_Simulator.dll` is a managed .NET assembly, not a native Windows-only DLL.
It can run on Linux as long as:

- .NET runtime is installed (`dotnet --info` works)
- Python uses `pythonnet` with `coreclr`
- OpenCL runtime/driver is available for `OpenCLKernel`

---

## Build the bridge

From `GB_Robotics_py`:

```bash
dotnet build dotnet_bridge/GBRobotics.PythonBridge.csproj -c Debug
```

Output folder: `dotnet_bridge/bin/Debug/net8.0/`

Required assemblies:

- `GBRobotics.PythonBridge.dll`
- `GB_Simulator.dll`
- `ParallelFlow.dll`
- `OpenCLKernel.dll`

---

## Install Python package

```bash
pip install -e .
```

## Install with uv

This project is PEP 621/517 compliant and works with `uv`.

```bash
uv venv
uv sync
```

Run CLI with uv:

```bash
uv run gb-robotics --help
uv run gb-robotics fk --joints "0,0.1,0.2,0,0,0"
uv run gb-robotics ik --matrix "1,0,0,300,0,1,0,0,0,0,1,400,0,0,0,1"
```

Or editable install:

```bash
uv pip install -e .
```

---

## CLI

After install:

```bash
gb-robotics --help
```

### Common flags

| Flag | Default | Description |
|---|---|---|
| `--assembly-dir` | `dotnet_bridge/bin/Debug/net8.0` | Path to .NET assemblies |
| `--robot-kind` | `lrmate200id7l` | Robot model |
| `--platform-id` | `0` | OpenCL platform id |
| `--device-id` | `0` | OpenCL device id |

### `fk` — Forward Kinematics

```bash
gb-robotics fk --joints "300,-20,40,10,-20,30"
gb-robotics fk --joints "300,-20,40,10,-20,30" --robot-id 0 --tool-id -1
```

### `ik` — Inverse Kinematics

```bash
gb-robotics ik --matrix "1,0,0,300,0,1,0,0,0,0,1,400,0,0,0,1"
```

### `view_joint_log` — 3-D Log Viewer

Parses a CSV log file, runs FK for every row, and opens an interactive 3-D viewer showing the TCP path.

```bash
gb-robotics view_joint_log log.csv lrmate200id7l \
    --joint-columns "5-11" \
    --robot-base "0,0,0,0,0,0" \
    --tool-frame "0,0,100,0,0,0"
```

**Arguments:**

- `log_file` — path to the CSV file
- `robot_kind` — robot model name (e.g. `lrmate200id7l`, `lr10ia10`)
- `--joint-columns` — columns containing joint values. Accepts:
  - a range: `"5-11"`
  - comma-separated 1-based indices: `"5,6,7,8,9,10"`
  - header names: `"J1,J2,J3,J4,J5,J6"`
- `--robot-base` — optional base frame as `x,y,z,w,p,r` (mm + degrees)
- `--tool-frame` — optional tool frame as `x,y,z,w,p,r` (mm + degrees)

**Viewer controls:**

- **Left drag** — orbit
- **Right drag / scroll** — zoom
- **Middle drag** — pan
- TCP path is colour-mapped blue (start) → red (end)
- XYZ triads shown at each pose (thinned automatically for large logs)

---

## Python API

```python
from gb_robotics_py import FanucSimulator

assembly_dir = "dotnet_bridge/bin/Debug/net8.0"

with FanucSimulator(assembly_dir, robot_kind="lrmate200id7l") as sim:
    fk = sim.fk([0.0, 0.1, 0.2, 0.0, 0.0, 0.0])
    print("FK out_of_range:", fk["out_of_range"])
    print("FK first link transform:", fk["link_transforms"][0])

    target = [fk["link_transforms"][-1]]
    ik = sim.ik(target)
    print("IK valid:", ik["is_valid"])
    print("IK joints:", ik["joints"])
```

### `FanucSimulator.fk(joint_values, robot_id=0, tool_id=-1, robot_base=None)`

- `joint_values` — 6-element list in **degrees** (converted to radians internally)
- `robot_base` — optional `[x, y, z, w, p, r]` base frame (mm + degrees)
- Returns:
  - `link_transforms` — list of 4×4 matrices (flattened, 16 values, row-major)
  - `tool_transforms` — optional tool transforms
  - `out_of_range` — per-joint boolean flags

### `FanucSimulator.ik(end_effector_transforms, robot_id=0)`

- `end_effector_transforms` — list of flattened 4×4 matrices (16 values each)
- Returns:
  - `joints` — list of 7 values per pose `[q1, q2, q3, q4, q5, q6, e1]`
  - `is_valid` — validity flag per pose

### `FanucSimulator.xyzwpr_to_dual_quaternion(xyzwpr, euler_order="ZYX")`

Converts a `[x, y, z, w, p, r]` pose (mm + degrees) to a dual quaternion.

- Returns a flat list of 8 floats: `[real.x, real.y, real.z, real.w, dual.x, dual.y, dual.z, dual.w]`

---

## Supported robots

| `robot_kind` | Description |
|---|---|
| `lrmate200id7l` | FANUC LR Mate 200iD/7L |
| `lr10ia10` | FANUC LR 10iA/10 |

---

## Debug Python + C\#

VS Code / Cursor debug configs are in `.vscode/launch.json`.

Use the compound config **`Python + C# (pythonnet)`** to:

1. Start `scripts/debug_fk_ik.py`
2. Attach CoreCLR to the Python process

The script pauses at startup (`input(...)`) so you can attach the C# debugger before FK/IK calls.

- Set Python breakpoints in `scripts/debug_fk_ik.py`
- Set C# breakpoints in `dotnet_bridge/FanucSimulatorBridge.cs`