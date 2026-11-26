from collections import defaultdict
from html import escape
import json
from pathlib import Path

from graphviz import Digraph


HERE = Path(__file__).resolve().parent


def render_block_diagram(
    config_path: str | Path = HERE / "configs/simplefoc_config.json",
    output_path: str | Path = HERE / "configs/pmsm_block_diagram.svg",
) -> Path:
    """Render a PMSM configuration to SVG using Graphviz and return its path."""
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    diagram = Digraph(
        name="pmsm_speed_control",
        graph_attr={
            "rankdir": "LR", "bgcolor": "#f8fafc", "pad": "0.32", "nodesep": "0.45",
            "ranksep": "0.50", "labelloc": "t", "label": "PMSM speed control",
            "fontname": "DejaVu Sans", "fontsize": "20", "fontcolor": "#0f172a",
        },
    )
    diagram.attr("node", shape="box", style="rounded,filled", fontname="DejaVu Sans",
                 penwidth="1.3", margin="0.16,0.12")
    diagram.attr("edge", fontname="DejaVu Sans", fontsize="10", arrowsize="0.75", penwidth="1.2")

    for block in config["blocks"]:
        name = block["name"]
        rate = block.get("rate_hz")
        if rate:
            clock = f"{rate / 1000:g} kHz"
        elif block["type"] in {"ConstantTorque", "ConstantCurrent"}:
            clock = "Constant"
        else:
            clock = "Continuous · RK4"
        diagram.node(
            name,
            label=f'<<B>{escape(name)}</B><BR/><FONT POINT-SIZE="10">{clock}</FONT>>',
            fillcolor="#dcfce7" if name == "pmsm" else "#ffffff",
            color="#15803d" if name == "pmsm" else "#94a3b8",
            fontcolor="#0f172a",
        )

    connections = defaultdict(list)
    for destination, source in config["connections"]:
        source_block, source_port = source.split(".", 1)
        destination_block, destination_port = destination.split(".", 1)
        port = source_port if source_port == destination_port else f"{source_port} → {destination_port}"
        connections[source_block, destination_block].append(port)

    for (source, destination), ports in connections.items():
        feedback = source == "pmsm"
        color = "#2563eb" if feedback else "#334155"
        diagram.edge(
            source,
            destination,
            label=", ".join(ports),
            color=color,
            fontcolor=color,
            constraint="false" if feedback else "true",
        )

    output = Path(output_path)
    output.write_bytes(diagram.pipe(format="svg"))
    return output


if __name__ == "__main__":
    render_block_diagram()
