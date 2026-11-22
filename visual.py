import math

import solara
from matplotlib.figure import Figure  # NOT pyplot: pyplot leaks figures in a server
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, Patch
from matplotlib.ticker import FuncFormatter
from mesa.visualization import SolaraViz, make_plot_component
from mesa.visualization.utils import update_counter

from model import TrafficModel, LANE_WIDTH

# If the road is no longer than this, show the WHOLE ring with a fixed camera,
# so you can watch cars leave the right edge and reappear on the left.
# If it's longer, show a window this wide that follows the first car.
VIEW_WIDTH = 360

BEHAVIOR_COLORS = {
    "DANGEROUS": "tab:red",
    "AVERAGE": "tab:blue",
    "CAUTIOUS": "tab:green",
}

# Single source of truth so the sliders and the initial model always agree.
DEFAULTS = {"length": 300, "num_lanes": 1, "num_drivers": 1, "t_step": 0.1}


def RoadView(model):
    """Top-down view: lanes as dashed lines, each car as a rectangle."""
    update_counter.get()  # re-render whenever the model steps

    road_len = model.length
    road_w = model.width
    agents = list(model.agents)

    if road_len <= VIEW_WIDTH:
        # Whole ring, fixed camera: wrap is visible at the left/right edges.
        xmin, xmax = 0.0, float(road_len)
    else:
        # Follow the first car; the road repeats beyond [0, road_len].
        center = agents[0].position[0] if agents else road_len / 2
        xmin, xmax = center - VIEW_WIDTH / 2, center + VIEW_WIDTH / 2
    view = xmax - xmin

    fig = Figure(figsize=(12, 1.4 + 0.8 * model.n_lanes))
    ax = fig.subplots()
    ax.set_facecolor("0.85")  # shoulder

    # Asphalt
    ax.add_patch(Rectangle((xmin, 0), view, road_w, facecolor="0.25", zorder=0))

    # Lane markings: solid road edges, dashed lines between lanes
    ax.axhline(0, color="white", lw=2, zorder=1)
    ax.axhline(road_w, color="white", lw=2, zorder=1)
    for i in range(1, model.n_lanes):
        ax.axhline(i * LANE_WIDTH, color="white", lw=1.5,
                   linestyle=(0, (8, 8)), zorder=1)

    # Seam where the ring wraps (x = 0 mod road_len)
    for k in range(math.ceil(xmin / road_len), math.floor(xmax / road_len) + 1):
        ax.axvline(k * road_len, color="gold", lw=1.5, linestyle=":", zorder=1)

    # Cars, with wrapped copies so a car crossing the seam shows on both sides.
    for a in agents:
        x, y = a.position
        color = BEHAVIOR_COLORS.get(a.behavior.name, "tab:gray")
        for dx in (-road_len, 0, road_len):
            cx = x + dx
            if cx + a.length / 2 < xmin or cx - a.length / 2 > xmax:
                continue
            ax.add_patch(Rectangle(
                (cx - a.length / 2, y - a.width / 2), a.length, a.width,
                facecolor=color, edgecolor="black", lw=1, zorder=2))

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(-0.5, road_w + 0.5)
    ax.set_aspect("auto")  # vertical axis is exaggerated so lanes are readable
    ax.set_yticks([(i + 0.5) * LANE_WIDTH for i in range(model.n_lanes)])
    ax.set_yticklabels([f"Lane {i}" for i in range(model.n_lanes)])
    # Label x ticks modulo the road length so past the seam they read 0, 1, 2...
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v % road_len:.0f}"))
    ax.set_xlabel(f"Position along road (m), road length = {road_len} m")
    ax.set_title(f"t = {model.sim_time:.1f} s")
    handles = [Patch(facecolor=c, edgecolor="black", label=n.title())
               for n, c in BEHAVIOR_COLORS.items()]
    handles.append(Line2D([0], [0], color="gold", linestyle=":", label="Wrap point"))
    ax.legend(handles=handles, loc="upper right", fontsize=8, ncol=4)

    return solara.FigureMatplotlib(fig, format="png", bbox_inches="tight")


model_params = {
    "rng": 42,
    "length": {"type": "SliderInt", "value": DEFAULTS["length"], "min": 100, "max": 1000, "step": 100, "label": "Road length (m)"},
    "num_lanes": {"type": "SliderInt", "value": DEFAULTS["num_lanes"], "min": 1, "max": 5, "step": 1, "label": "Num Lanes"},
    "num_drivers": {"type": "SliderInt", "value": DEFAULTS["num_drivers"], "min": 1, "max": 100, "step": 1, "label": "Num Drivers"},
    "t_step": {"type": "SliderFloat", "value": DEFAULTS["t_step"], "min": 0.01, "max": 1.0, "step": 0.01, "label": "Time Step (s)"},
}

traffic_model = TrafficModel(rng=42, **DEFAULTS)

SpeedPlot = make_plot_component("mean_speed")

page = SolaraViz(
    traffic_model,
    components=[RoadView, SpeedPlot],
    model_params=model_params,
    name="Ring-road with no ramps",
    play_interval=100,
)
page  # Solara looks for a top-level `page`