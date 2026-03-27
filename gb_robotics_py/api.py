from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Sequence


def _load_clr():
    from pythonnet import load

    try:
        load("coreclr")
    except Exception:
        # Runtime may already be initialized by host process.
        pass

    import clr  # type: ignore

    return clr


def _to_python(value):
    if value is None:
        return None
    try:
        return [_to_python(item) for item in value]
    except TypeError:
        return value


class FanucSimulator:
    def __init__(
        self,
        assembly_dir: str | Path,
        robot_kind: str = "lrmate200id7l",
        platform_id: int = 0,
        device_id: int = 0,
    ) -> None:
        self._assembly_dir = Path(assembly_dir).expanduser().resolve()
        self._load_assemblies(self._assembly_dir)

        from GBRobotics.PythonBridge import FanucSimulatorBridge  # type: ignore

        self._bridge = FanucSimulatorBridge(robot_kind, int(platform_id), int(device_id))

    @staticmethod
    def _load_assemblies(assembly_dir: Path) -> None:
        if not assembly_dir.exists():
            raise FileNotFoundError(f"Assembly directory does not exist: {assembly_dir}")

        clr = _load_clr()
        required = [
            "ParallelFlow.dll",
            "OpenCLKernel.dll",
            "GB_Simulator.dll",
            "GBRobotics.PythonBridge.dll",
        ]

        for filename in required:
            dll_path = assembly_dir / filename
            if not dll_path.exists():
                raise FileNotFoundError(f"Required assembly not found: {dll_path}")
            clr.AddReference(str(dll_path))

    def fk(self, joint_values: Sequence[float], robot_id: int = 0, tool_id: int = -1, robot_base: Sequence[float] | None = None) -> dict:
        # converting the robot_base from xyz + rpy to a dual quaternion
        if robot_base is not None:
            robot_base_dq = self.xyzwpr_to_dual_quaternion(robot_base)
        
        result = self._bridge.Fk([float(v) for v in joint_values], int(robot_id), int(tool_id), robot_base_dq if robot_base is not None else None)
        return {
            "link_transforms": _to_python(result.LinkTransforms),
            "tool_transforms": _to_python(result.ToolTransforms),
            "out_of_range": _to_python(result.OutOfRange),
        }

    def ik(self, end_effector_transforms: Iterable[Sequence[float]], robot_id: int = 0) -> dict:
        transforms = [[float(v) for v in matrix] for matrix in end_effector_transforms]
        result = self._bridge.Ik(transforms, int(robot_id))
        return {
            "joints": _to_python(result.Joints),
            "is_valid": _to_python(result.IsValid),
        }
    
    def xyzwpr_to_dual_quaternion(
    xyzwpr: Sequence[float],
    euler_order: str = "ZYX",
    ) -> list[float]:
        """
        Convert a list of 6 values [x, y, z, w, p, r] (XYZWPR) to a dual quaternion.

        Parameters
        ----------
        xyzwpr : Sequence[float]
            Six values: x, y, z in millimetres; w, p, r in degrees (Fanuc convention).
        assembly_dir : str | Path
            Path to the directory containing ParallelFlow.dll and GB_Simulator.dll.
        euler_order : str
            Euler angle rotation order passed to QuaternionFromEulerAngles (default "ZYX").

        Returns
        -------
        list[float]
            The dual quaternion as a flat list of 8 floats:
            [real.x, real.y, real.z, real.w, dual.x, dual.y, dual.z, dual.w]
        """
        if len(xyzwpr) != 6:
            raise ValueError(f"xyzwpr must have exactly 6 values, got {len(xyzwpr)}.")

        #load_assemblies(assembly_dir)

        ## Import .NET types from ParallelFlow
        #from ParallelFlow.LinearAlgebra import (  # type: ignore
        #    Vector4_d,
        #    Quaternion_d,
        #    DualQuaternion_d,
        #)
        from ParallelFlow.Types import (  # type: ignore
            Vector4_d,
            Quaternion_d,
            DualQuaternion_d,
            IQuaternionExtensions,
            DualQuaternionExtensions,
        )

        x, y, z, w, p, r = [float(v) for v in xyzwpr]

        # Convert W, P, R from degrees to radians
        w_rad = math.radians(w) 
        p_rad = math.radians(p)
        r_rad = math.radians(r)

        # Build euler angle vector (w, p, r, 0) — matches C# example
        euler_angles = Vector4_d(w_rad, p_rad, r_rad, 0.0)

        # Build rotation quaternion from euler angles
        q = IQuaternionExtensions.QuaternionFromEulerAngles[
            float, Vector4_d, Quaternion_d
        ](euler_angles, euler_order)

        # Build translation vector (x, y, z)
        translation = Vector4_d(x, y, z, 0.0)

        # Combine into dual quaternion
        return DualQuaternionExtensions.FromRotationAndTranslation[
            float, Vector4_d, Quaternion_d, DualQuaternion_d
        ]

    def close(self) -> None:
        if self._bridge is not None:
            self._bridge.Dispose()
            self._bridge = None

    def __enter__(self) -> "FanucSimulator":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
