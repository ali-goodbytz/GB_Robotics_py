from pathlib import Path

from gb_robotics_py.api import _load_clr

# Host CoreCLR on import so "Attach to python.exe" (even if started in parallel with the script)
# sees a managed runtime before FanucSimulator loads the bridge DLL.
_load_clr()

from gb_robotics_py import FanucSimulator


def main() -> None:
    assembly_dir = Path(__file__).resolve().parents[1] / "dotnet_bridge" / "bin" / "Debug" / "net8.0"
    print(f"Assembly dir: {assembly_dir}")

    # Put a Python breakpoint on the next line (only when using the debugpy config without noDebug).
    input("Attach C# debugger to python.exe, then press Enter...")

    with FanucSimulator(assembly_dir, robot_kind="lrmate200id7l") as sim:
        joints = [300, -20, 40, 10, -20, 30]

        # Put a Python breakpoint on the next line.
        fk = sim.fk(joints)
        print("FK out_of_range:", fk["out_of_range"])
        print("FK link count:", len(fk["link_transforms"]))
        print("FK link transforms:", fk["link_transforms"])

        target = [fk["link_transforms"][-1]]

        # Put a Python breakpoint on the next line.
        ik = sim.ik(target)
        print("IK is_valid:", ik["is_valid"])
        print("IK first solution:", ik["joints"][0])


if __name__ == "__main__":
    main()
