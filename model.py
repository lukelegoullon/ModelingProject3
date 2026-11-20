import numpy as np
import solara
from matplotlib.figure import Figure   # NOT pyplot: pyplot leaks figures in a server
from mesa.visualization import SolaraViz, make_plot_component
from mesa.visualization.utils import update_counter

import driver


# ---------------------------------------------------------------- road view
@solara.component
def RoadView(model):
    update_counter.get()  # required, otherwise the component won't redraw each step
    solara.FigureMatplotlib(fig)

# ---------------------------------------------------------------- live space-time diagram
@solara.component
def SpaceTime(model, last=300):
    update_counter.get()
    solara.FigureMatplotlib(fig)

# ---------------------------------------------------------------- charts from the DataCollector
SpeedPlot = make_plot_component("mean_speed")

page = SolaraViz(
    model,
    components=[RoadView, SpaceTime, SpeedPlot],
    model_params=model_params,
    name="Ring-road with no ramps",
    play_interval=100,   # ms between steps; lower = faster
)
page  # Solara looks for a top-level `page`
if __name__ == "__main__":
    import subprocess
    import sys

    # Launch the Solara server on this file; it opens your default browser automatically.
    # Because solara imports the file as a module, this block doesn't run again (no recursion).
    subprocess.run([sys.executable, "-m", "solara", "run", __file__])