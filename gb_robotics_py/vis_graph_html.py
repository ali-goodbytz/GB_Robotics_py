"""Write a self-contained Plotly HTML viewer for motion graphs."""

from __future__ import annotations

import json
from pathlib import Path

from .vis_graph_data import EdgeScene, NodeScene, RegisterScene


def _stable_palette(keys: list[str]) -> dict[str, str]:
    """Assign a distinct hex color to each key (stable order)."""
    palette = [
        "#4FC3F7",
        "#FF6B6B",
        "#FFD700",
        "#A5D6A7",
        "#CE93D8",
        "#FFAB40",
        "#80DEEA",
        "#EF9A9A",
        "#C5E1A5",
        "#B0BEC5",
        "#29B6F6",
        "#FFCA28",
        "#AB47BC",
        "#66BB6A",
        "#78909C",
    ]
    out: dict[str, str] = {}
    for i, k in enumerate(keys):
        out[k] = palette[i % len(palette)]
    return out


def _node_label_key(node: NodeScene, attr: str) -> str:
    raw = str(getattr(node, attr, None) or "").strip()
    return raw if raw else "(none)"


def _node_color_lists(nodes: list[NodeScene]) -> dict[str, list[str]]:
    """Parallel color arrays per node for source / group / node_type coloring."""
    out: dict[str, list[str]] = {}
    for mode, attr in (
        ("source", "source"),
        ("group", "group"),
        ("type", "node_type"),
    ):
        keys = sorted({_node_label_key(n, attr) for n in nodes})
        pal = _stable_palette(keys)
        out[mode] = [_node_label_key(n, attr) for n in nodes]
        out[mode] = [pal[k] for k in out[mode]]
    return out


def _node_marker_sizes(nodes: list[NodeScene]) -> list[float]:
    """Home / register style nodes are drawn larger than typical graph nodes."""
    base = 9.0
    big = 18.0
    return [big if n.emphasis else base for n in nodes]


def _categorical_maps(
    edges: list[EdgeScene],
) -> tuple[dict[str, str], dict[str, str], dict[int, str], dict[int, str]]:
    et_keys = sorted({e.edge_type or "(none)" for e in edges})
    ps_keys = sorted({e.part_state or "(none)" for e in edges})
    et_palette = _stable_palette(et_keys)
    ps_palette = _stable_palette(ps_keys)
    edge_color_et: dict[int, str] = {}
    edge_color_ps: dict[int, str] = {}
    for e in edges:
        edge_color_et[e.id] = et_palette[e.edge_type or "(none)"]
        edge_color_ps[e.id] = ps_palette[e.part_state or "(none)"]
    return et_palette, ps_palette, edge_color_et, edge_color_ps


def _toolbar_post_script(ui_cfg: dict) -> str:
    """Plotly post_script: edge/node recolor controls and save HTML."""
    cfg_json = json.dumps(ui_cfg)
    js_lines = [
        f"    window.__VIS_GRAPH_UI__ = {cfg_json};",
        "    (function() {",
        '        var gd = document.querySelector(".js-plotly-plot") ||',
        '            document.querySelector(".plotly-graph-div");',
        "        if (!gd || !window.__VIS_GRAPH_UI__) return;",
        "        var root = window.__VIS_GRAPH_UI__;",
        "        var ec = root.edges;",
        "        var nc = root.nodes;",
        "        function edgeColorsFor(mode) {",
        "            var map = mode === 'part_state' ? ec.colorByPartState : ec.colorByEdgeType;",
        "            return ec.edgeIds.map(function(id) { return map[String(id)]; });",
        "        }",
        "        function applyEdgeMode(mode) {",
        "            Plotly.restyle(gd, {'line.color': edgeColorsFor(mode)}, ec.edgeTraceIndices);",
        "        }",
        "        function applyNodeColorMode(mode) {",
        "            var cols = nc.colors[mode] || nc.colors.source;",
        "            Plotly.restyle(gd, {'marker.color': [cols], 'marker.size': [nc.sizes]}, [nc.traceIndex]);",
        "        }",
        "        var bar = document.createElement('div');",
        "        bar.style.cssText = [",
        "            'position:fixed;top:10px;right:10px;z-index:99999;',",
        "            'display:flex;gap:8px;flex-direction:column;',",
        "            'align-items:flex-end;font-family:system-ui,sans-serif;',",
        "        ].join('');",
        "        var saveBtn = document.createElement('button');",
        "        saveBtn.innerText = 'Save view as HTML';",
        "        saveBtn.title = (",
        "            'Downloads self-contained HTML (large with embedded Plotly).'",
        "        );",
        "        saveBtn.style.cssText = [",
        "            'padding:8px 14px;background:#4FC3F7;color:#000;',",
        "            'border:none;border-radius:6px;cursor:pointer;',",
        "            'font-size:13px;font-weight:bold;',",
        "        ].join('');",
        "        saveBtn.onclick = function() {",
        "            var html = document.documentElement.outerHTML;",
        "            var blob = new Blob([html], {type: 'text/html'});",
        "            var a = document.createElement('a');",
        "            a.href = URL.createObjectURL(blob);",
        "            a.download = 'motion_graph_vis.html';",
        "            a.click();",
        "        };",
        "        var rowEdge = document.createElement('div');",
        "        rowEdge.style.cssText = [",
        "            'display:flex;align-items:center;gap:6px;',",
        "            'background:#2a2a2a;padding:8px 10px;',",
        "            'border-radius:8px;border:1px solid #555;',",
        "        ].join('');",
        "        var labE = document.createElement('label');",
        "        labE.innerText = 'Edge colors';",
        "        labE.style.cssText = 'color:#fff;font-size:12px;margin-right:4px;';",
        "        var selE = document.createElement('select');",
        "        selE.style.cssText = [",
        "            'padding:6px 10px;border-radius:6px;',",
        "            'border:1px solid #555;background:#2a2a2a;color:#fff;',",
        "        ].join('');",
        "        [['edge_type','By edge_type'],['part_state','By part_state']].forEach(function(opt) {",
        "            var o = document.createElement('option');",
        "            o.value = opt[0]; o.text = opt[1]; selE.appendChild(o);",
        "        });",
        "        selE.onchange = function() { applyEdgeMode(selE.value); };",
        "        rowEdge.appendChild(labE);",
        "        rowEdge.appendChild(selE);",
        "        var rowNode = document.createElement('div');",
        "        rowNode.style.cssText = rowEdge.style.cssText;",
        "        var labN = document.createElement('label');",
        "        labN.innerText = 'Node colors';",
        "        labN.style.cssText = labE.style.cssText;",
        "        var selN = document.createElement('select');",
        "        selN.style.cssText = selE.style.cssText;",
        "        [['source','By source'],['group','By group'],['type','By type (node_type)']].forEach(function(opt) {",
        "            var o = document.createElement('option');",
        "            o.value = opt[0]; o.text = opt[1]; selN.appendChild(o);",
        "        });",
        "        selN.value = nc.initialColorBy || 'source';",
        "        selN.onchange = function() { applyNodeColorMode(selN.value); };",
        "        rowNode.appendChild(labN);",
        "        rowNode.appendChild(selN);",
        "        bar.appendChild(saveBtn);",
        "        bar.appendChild(rowEdge);",
        "        bar.appendChild(rowNode);",
        "        document.body.appendChild(bar);",
        "    })();",
    ]
    return "\n".join(js_lines)


def write_vis_graph_html(
    *,
    path: Path,
    nodes: list[NodeScene],
    edges: list[EdgeScene],
    registers: list[RegisterScene],
    graph_title: str = "Motion graph (TCP)",
    node_color_by: str = "source",
) -> None:
    """Write interactive HTML with Plotly (embedded JS) and UI controls."""
    try:
        import plotly.graph_objects as go
    except ImportError as exc:  # pragma: no cover - exercised via test optional
        raise ImportError(
            "Plotly is required for vis-graph. Install with: pip install plotly"
        ) from exc

    mode = (node_color_by or "source").strip().lower()
    if mode not in ("source", "group", "type"):
        raise ValueError(
            "node_color_by must be one of: source, group, type (maps to node_type)."
        )

    node_color_lists = _node_color_lists(nodes)
    node_sizes = _node_marker_sizes(nodes)
    node_colors_initial = node_color_lists[mode]

    traces: list = []
    traces.append(
        go.Scatter3d(
            x=[n.x for n in nodes],
            y=[n.y for n in nodes],
            z=[n.z for n in nodes],
            mode="markers+text",
            name="Nodes",
            text=[n.label for n in nodes],
            textposition="top center",
            customdata=[
                [n.id, n.source, n.group, n.node_type, n.kind] for n in nodes
            ],
            hovertemplate=(
                "%{text}<br>id: %{customdata[0]}<br>"
                "source: %{customdata[1]}<br>group: %{customdata[2]}<br>"
                "type: %{customdata[3]}<br>kind: %{customdata[4]}<br>"
                "X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>"
            ),
            marker=dict(
                size=node_sizes,
                color=node_colors_initial,
                line=dict(width=1, color="#222"),
            ),
        )
    )

    _, _, edge_color_et, edge_color_ps = _categorical_maps(edges)
    edge_trace_indices: list[int] = []
    edge_ids_ordered: list[int] = []

    for i, e in enumerate(edges):
        color = edge_color_et[e.id]
        traces.append(
            go.Scatter3d(
                x=e.xs,
                y=e.ys,
                z=e.zs,
                mode="lines",
                name=f"edge {e.id}",
                line=dict(width=5, color=color),
                showlegend=False,
                hovertemplate=(
                    f"edge {e.id}<br>edge_type: {e.edge_type}<br>"
                    f"part_state: {e.part_state}<br>"
                    "X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>"
                ),
            )
        )
        edge_trace_indices.append(1 + i)
        edge_ids_ordered.append(e.id)

    if registers:
        from itertools import groupby

        sorted_regs = sorted(registers, key=lambda r: r.group)
        for group, members_iter in groupby(sorted_regs, key=lambda r: r.group):
            members = list(members_iter)
            traces.append(
                go.Scatter3d(
                    x=[m.x for m in members],
                    y=[m.y for m in members],
                    z=[m.z for m in members],
                    mode="markers+text",
                    name=f"Registers [{group}]",
                    text=[m.label for m in members],
                    textposition="top center",
                    marker=dict(
                        size=22,
                        line=dict(width=2, color="white"),
                    ),
                )
            )

    fig = go.Figure(data=traces)
    fig.update_layout(
        title=graph_title,
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
        margin=dict(l=0, r=0, t=50, b=0),
    )

    ui_cfg = {
        "edges": {
            "edgeTraceIndices": edge_trace_indices,
            "edgeIds": edge_ids_ordered,
            "colorByEdgeType": {str(k): v for k, v in edge_color_et.items()},
            "colorByPartState": {str(k): v for k, v in edge_color_ps.items()},
        },
        "nodes": {
            "traceIndex": 0,
            "sizes": node_sizes,
            "colors": {
                "source": node_color_lists["source"],
                "group": node_color_lists["group"],
                "type": node_color_lists["type"],
            },
            "initialColorBy": mode,
        },
    }

    post_script = _toolbar_post_script(ui_cfg)

    path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(
        str(path),
        include_plotlyjs=True,
        post_script=post_script,
    )
