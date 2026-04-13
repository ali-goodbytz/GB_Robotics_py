"""Build 3-D scene payloads for motion-graph visualization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

from robot_profiler.models.motion_graph import MotionGraph
from robot_profiler.models.node import Node


def parse_xyzwpr(text: str) -> list[float]:
    """Parse comma-separated XYZWPR (mm, degrees) into six floats."""
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != 6:
        raise ValueError(f"XYZWPR must have exactly 6 values, got {len(parts)}.")
    return [float(p) for p in parts]


def matrix_row_major_to_xyz(matrix_flat: Sequence[float]) -> tuple[float, float, float]:
    """Extract translation (x, y, z) from a flat row-major 4x4 transform."""
    return float(matrix_flat[3]), float(matrix_flat[7]), float(matrix_flat[11])


JointToTcp = Callable[[Sequence[float]], tuple[float, float, float] | None]


def _node_emphasis_home_or_register(node: Node) -> bool:
    """Larger markers for home / register style graph nodes (kind, type, source, label)."""
    k = (node.kind or "").strip().lower()
    nt = (node.node_type or "").strip().lower()
    if k in ("home", "register") or nt in ("home", "register"):
        return True
    blob = " ".join(
        str(p)
        for p in (node.source, node.group, node.label)
        if p
    ).lower()
    return "home" in blob or "register" in blob


def _edge_type_label(edge_type: object) -> str | None:
    """Support ``Edge.edge_type`` as ``str`` (current robot_profiler) or enum-like ``.value``."""
    if edge_type is None:
        return None
    if isinstance(edge_type, str):
        return edge_type
    val = getattr(edge_type, "value", edge_type)
    return str(val) if val is not None else None


@dataclass(slots=True)
class NodeScene:
    id: int
    label: str
    x: float
    y: float
    z: float
    source: str = ""
    group: str = ""
    node_type: str = ""
    kind: str = ""
    emphasis: bool = False


@dataclass(slots=True)
class EdgeScene:
    id: int
    start_node_id: int
    end_node_id: int
    edge_type: str | None
    part_state: str | None
    xs: list[float]
    ys: list[float]
    zs: list[float]


@dataclass(slots=True)
class RegisterScene:
    label: str
    group: str
    x: float
    y: float
    z: float


def _node_by_id(graph: MotionGraph) -> dict[int, Node]:
    return {n.id: n for n in graph.nodes}


def build_scene(
    graph: MotionGraph,
    joint_to_tcp: JointToTcp,
    *,
    registers_raw: list[dict] | None = None,
) -> tuple[list[NodeScene], list[EdgeScene], list[RegisterScene], list[str]]:
    """
    Compute TCP polylines and node positions using ``joint_to_tcp``.

    Returns (nodes, edges, registers, warnings).
    """
    warnings: list[str] = []
    nodes_out: list[NodeScene] = []
    for node in graph.nodes:
        xyz = joint_to_tcp(node.joint_pose.joints)
        if xyz is None:
            warnings.append(f"FK failed for node {node.id} ({node.label!r}).")
            continue
        nodes_out.append(
            NodeScene(
                id=node.id,
                label=node.label,
                x=xyz[0],
                y=xyz[1],
                z=xyz[2],
                source=node.source or "",
                group=node.group or "",
                node_type=node.node_type or "",
                kind=node.kind or "",
                emphasis=_node_emphasis_home_or_register(node),
            )
        )

    id_to_node = _node_by_id(graph)
    edges_out: list[EdgeScene] = []
    for edge in graph.edges:
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        if edge.samples:
            for sample in edge.samples:
                xyz = joint_to_tcp(sample.joint_pose.joints)
                if xyz is None:
                    warnings.append(
                        f"FK failed for edge {edge.id} sample row {sample.row_number}."
                    )
                    continue
                xs.append(xyz[0])
                ys.append(xyz[1])
                zs.append(xyz[2])
        else:
            sn = id_to_node.get(edge.start_node_id)
            en = id_to_node.get(edge.end_node_id)
            if sn is not None:
                a = joint_to_tcp(sn.joint_pose.joints)
                if a:
                    xs.append(a[0])
                    ys.append(a[1])
                    zs.append(a[2])
            if en is not None:
                b = joint_to_tcp(en.joint_pose.joints)
                if b:
                    xs.append(b[0])
                    ys.append(b[1])
                    zs.append(b[2])
        if len(xs) < 2:
            warnings.append(
                f"Edge {edge.id} has fewer than two FK points after sampling; skipped."
            )
            continue
        et = _edge_type_label(edge.edge_type)
        edges_out.append(
            EdgeScene(
                id=edge.id,
                start_node_id=edge.start_node_id,
                end_node_id=edge.end_node_id,
                edge_type=et,
                part_state=edge.part_state,
                xs=xs,
                ys=ys,
                zs=zs,
            )
        )

    reg_out: list[RegisterScene] = []
    if registers_raw:
        for reg in registers_raw:
            label = str(reg.get("label", "?"))
            group = str(reg.get("group", "registers"))
            joints = reg.get("joints")
            if not isinstance(joints, list):
                warnings.append(f"Register {label!r}: missing joints list.")
                continue
            try:
                jt = [float(v) for v in joints]
            except (TypeError, ValueError):
                warnings.append(f"Register {label!r}: invalid joints.")
                continue
            xyz = joint_to_tcp(jt)
            if xyz is None:
                warnings.append(f"FK failed for register {label!r}.")
                continue
            reg_out.append(
                RegisterScene(label=label, group=group, x=xyz[0], y=xyz[1], z=xyz[2])
            )

    return nodes_out, edges_out, reg_out, warnings
