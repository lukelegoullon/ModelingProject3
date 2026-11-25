# import math

# import solara
# from matplotlib.figure import Figure  # NOT pyplot: pyplot leaks figures in a server
# from matplotlib.lines import Line2D
# from matplotlib.patches import Rectangle, Patch
# from matplotlib.ticker import FuncFormatter
# from mesa.visualization import SolaraViz, make_plot_component
# from mesa.visualization.utils import update_counter

# from model import TrafficModel, LANE_WIDTH

# # If the road is no longer than this, show the WHOLE ring with a fixed camera,
# # so you can watch cars leave the right edge and reappear on the left.
# # If it's longer, show a window this wide that follows the first car.
# VIEW_WIDTH = 600

# BEHAVIOR_COLORS = {
#     "DANGEROUS": "tab:red",
#     "AVERAGE": "tab:blue",
#     "CAUTIOUS": "tab:green",
# }

# # Single source of truth so the sliders and the initial model always agree.
# DEFAULTS = {"length": 300, "num_lanes": 1, "num_drivers": 1, "t_step": 0.1}


# def RoadView(model):
#     """Top-down view: lanes as dashed lines, each car as a rectangle."""
#     update_counter.get()  # re-render whenever the model steps

#     road_len = model.length
#     road_w = model.width
#     agents = list(model.agents)

#     if road_len <= VIEW_WIDTH:
#         # Whole ring, fixed camera: wrap is visible at the left/right edges.
#         xmin, xmax = 0.0, float(road_len)
#     else:
#         # Follow the first car; the road repeats beyond [0, road_len].
#         center = agents[0].position[0] if agents else road_len / 2
#         xmin, xmax = center - VIEW_WIDTH / 2, center + VIEW_WIDTH / 2
#     view = xmax - xmin

#     fig = Figure(figsize=(12, 1.4 + 0.8 * model.n_lanes))
#     ax = fig.subplots()
#     ax.set_facecolor("0.85")  # shoulder

#     # Asphalt
#     ax.add_patch(Rectangle((xmin, 0), view, road_w, facecolor="0.25", zorder=0))

#     # Lane markings: solid road edges, dashed lines between lanes
#     ax.axhline(0, color="white", lw=2, zorder=1)
#     ax.axhline(road_w, color="white", lw=2, zorder=1)
#     for i in range(1, model.n_lanes):
#         ax.axhline(i * LANE_WIDTH, color="white", lw=1.5,
#                    linestyle=(0, (8, 8)), zorder=1)

#     # Seam where the ring wraps (x = 0 mod road_len)
#     for k in range(math.ceil(xmin / road_len), math.floor(xmax / road_len) + 1):
#         ax.axvline(k * road_len, color="gold", lw=1.5, linestyle=":", zorder=1)

#     # Cars, with wrapped copies so a car crossing the seam shows on both sides.
#     for a in agents:
#         x, y = a.position
#         color = BEHAVIOR_COLORS.get(a.behavior.name, "tab:gray")
#         for dx in (-road_len, 0, road_len):
#             cx = x + dx
#             if cx + a.length / 2 < xmin or cx - a.length / 2 > xmax:
#                 continue
#             ax.add_patch(Rectangle(
#                 (cx - a.length / 2, y - a.width / 2), a.length, a.width,
#                 facecolor=color, edgecolor="black", lw=1, zorder=2))

#     ax.set_xlim(xmin, xmax)
#     ax.set_ylim(-0.5, road_w + 0.5)
#     ax.set_aspect("auto")  # vertical axis is exaggerated so lanes are readable
#     ax.set_yticks([(i + 0.5) * LANE_WIDTH for i in range(model.n_lanes)])
#     ax.set_yticklabels([f"Lane {i}" for i in range(model.n_lanes)])
#     # Label x ticks modulo the road length so past the seam they read 0, 1, 2...
#     ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v % road_len:.0f}"))
#     ax.set_xlabel(f"Position along road (m), road length = {road_len} m")
#     ax.set_title(f"t = {model.sim_time:.1f} s")
#     handles = [Patch(facecolor=c, edgecolor="black", label=n.title())
#                for n, c in BEHAVIOR_COLORS.items()]
#     handles.append(Line2D([0], [0], color="gold", linestyle=":", label="Wrap point"))
#     ax.legend(handles=handles, loc="upper right", fontsize=8, ncol=4)

#     return solara.FigureMatplotlib(fig, format="png", bbox_inches="tight")


# model_params = {
#     "rng": 42,
#     "length": {"type": "SliderInt", "value": DEFAULTS["length"], "min": 100, "max": 1000, "step": 100, "label": "Road length (m)"},
#     "num_lanes": {"type": "SliderInt", "value": DEFAULTS["num_lanes"], "min": 1, "max": 5, "step": 1, "label": "Num Lanes"},
#     "num_drivers": {"type": "SliderInt", "value": DEFAULTS["num_drivers"], "min": 1, "max": 100, "step": 1, "label": "Num Drivers"},
#     "t_step": {"type": "SliderFloat", "value": DEFAULTS["t_step"], "min": 0.01, "max": 1.0, "step": 0.01, "label": "Time Step (s)"},
# }

# traffic_model = TrafficModel(rng=42, **DEFAULTS)

# SpeedPlot = make_plot_component("mean_speed")

# page = SolaraViz(
#     traffic_model,
#     components=[RoadView, SpeedPlot],
#     model_params=model_params,
#     name="Ring-road with no ramps",
#     play_interval=100,
# )
# page  # Solara looks for a top-level `page`
import math

import numpy as np
import solara
from matplotlib import colormaps
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.figure import Figure  # NOT pyplot: pyplot leaks figures in a server
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, Patch
from matplotlib.ticker import FuncFormatter
from mesa.visualization import SolaraViz, make_plot_component
from mesa.visualization.utils import update_counter

from driver import SPEED_LIMIT
from model import TrafficModel, LANE_WIDTH

FOLLOW_WIDTH = 150      # meters visible in the follow-car view
SPACETIME_WINDOW = 120  # seconds of history in the space-time diagram

CMAP = colormaps["RdYlGn"]                       # slow = red, fast = green
NORM = Normalize(vmin=0, vmax=SPEED_LIMIT + 15)  # fastest behavior's desired speed

BEHAVIOR_EDGE = {"DANGEROUS": "red", "AVERAGE": "black", "CAUTIOUS": "blue"}

DEFAULTS = {"length": 500, "num_lanes": 1, "num_drivers": 30, "t_step": 0.1}


def draw_road(ax, model, xmin, xmax, min_car_len=0.0, label_ids=False):
    """Draw asphalt, lane lines, wrap seams and speed-colored cars into ax."""
    road_len, road_w = model.length, model.width
    view = xmax - xmin

    ax.set_facecolor("0.85")
    ax.add_patch(Rectangle((xmin, 0), view, road_w, facecolor="0.25", zorder=0))
    ax.axhline(0, color="white", lw=2, zorder=1)
    ax.axhline(road_w, color="white", lw=2, zorder=1)
    for i in range(1, model.n_lanes):
        ax.axhline(i * LANE_WIDTH, color="white", lw=1.5,
                   linestyle=(0, (8, 8)), zorder=1)

    # Seam where the ring wraps
    for k in range(math.ceil(xmin / road_len), math.floor(xmax / road_len) + 1):
        ax.axvline(k * road_len, color="gold", lw=1.5, linestyle=":", zorder=1)

    k_lo = math.floor(xmin / road_len) - 1
    k_hi = math.ceil(xmax / road_len) + 1
    for idx, a in enumerate(model.agents):
        x, y = a.position
        disp_len = max(a.length, min_car_len)       # exaggerate if zoomed out
        edge = BEHAVIOR_EDGE.get(a.behavior.name, "gray")
        color = CMAP(NORM(a.velo))
        for k in range(k_lo, k_hi + 1):             # wrapped copies
            cx = x + k * road_len
            if cx + disp_len / 2 < xmin or cx - disp_len / 2 > xmax:
                continue
            ax.add_patch(Rectangle(
                (cx - disp_len / 2, y - a.width / 2), disp_len, a.width,
                facecolor=color, edgecolor=edge, lw=2 if idx == 0 else 1,
                zorder=3 if idx == 0 else 2))
            if label_ids and idx == 0:
                ax.text(cx, y, "0", ha="center", va="center",
                        fontsize=8, fontweight="bold", zorder=4)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(-0.5, road_w + 0.5)
    ax.set_aspect("auto")
    ax.set_yticks([(i + 0.5) * LANE_WIDTH for i in range(model.n_lanes)])
    ax.set_yticklabels([f"Lane {i}" for i in range(model.n_lanes)])
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v % road_len:.0f}"))


def _legend(ax):
    handles = [Patch(facecolor="white", edgecolor=c, lw=2, label=n.title())
               for n, c in BEHAVIOR_EDGE.items()]
    handles.append(Line2D([0], [0], color="gold", linestyle=":", label="Wrap point"))
    ax.legend(handles=handles, loc="upper right", fontsize=8, ncol=4)


def FullRoadView(model):
    """Whole ring, fixed camera."""
    update_counter.get()
    fig = Figure(figsize=(12, 1.8 + 0.6 * model.n_lanes))
    ax = fig.subplots()
    draw_road(ax, model, 0.0, float(model.length),
              min_car_len=0.012 * model.length, label_ids=True)
    ax.set_xlabel(f"Position (m), road length = {model.length} m "
                  f"(cars enlarged for visibility)")
    ax.set_title(f"Full road, t = {model.sim_time:.1f} s")
    fig.colorbar(ScalarMappable(norm=NORM, cmap=CMAP), ax=ax,
                 pad=0.01, fraction=0.03, label="speed (m/s)")
    _legend(ax)
    return solara.FigureMatplotlib(fig, format="png", bbox_inches="tight")


def FollowView(model):
    """Zoomed window that tracks car 0 (outlined thicker, labeled '0')."""
    update_counter.get()
    agents = list(model.agents)
    center = agents[0].position[0] if agents else model.length / 2
    fig = Figure(figsize=(12, 1.8 + 0.6 * model.n_lanes))
    ax = fig.subplots()
    draw_road(ax, model, center - FOLLOW_WIDTH / 2, center + FOLLOW_WIDTH / 2,
              label_ids=True)
    ax.set_xlabel("Position (m), centered on car 0")
    if agents:
        a0 = agents[0]
        ax.set_title(f"Following car 0 ({a0.behavior.name.title()}): "
                     f"{a0.velo:.1f} m/s, gap {a0.gap:.1f} m")
    fig.colorbar(ScalarMappable(norm=NORM, cmap=CMAP), ax=ax,
                 pad=0.01, fraction=0.03, label="speed (m/s)")
    return solara.FigureMatplotlib(fig, format="png", bbox_inches="tight")


def SpaceTimeView(model):
    """Position vs time, colored by speed. Jams = red bands leaning backward."""
    update_counter.get()
    fig = Figure(figsize=(12, 4))
    ax = fig.subplots()

    hist = list(model.history)
    if len(hist) > 1:
        t_now = hist[-1][0]
        hist = [h for h in hist if h[0] >= t_now - SPACETIME_WINDOW]
        hist = hist[::max(1, len(hist) // 400)]      # thin out for speed
        ts = np.concatenate([np.full(len(x), t) for t, x, _ in hist])
        xs = np.concatenate([x for _, x, _ in hist])
        vs = np.concatenate([v for _, _, v in hist])
        ax.scatter(ts, xs, c=vs, s=4, cmap=CMAP, norm=NORM, linewidths=0)
        ax.set_xlim(max(0, t_now - SPACETIME_WINDOW), max(t_now, 1))

    ax.set_ylim(0, model.length)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Space-time diagram (red bands = slow traffic)")
    fig.colorbar(ScalarMappable(norm=NORM, cmap=CMAP), ax=ax,
                 pad=0.01, fraction=0.03, label="speed (m/s)")
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
    components=[FullRoadView, FollowView, SpaceTimeView, SpeedPlot],
    model_params=model_params,
    name="Ring-road with no ramps",
    play_interval=100,
)
page