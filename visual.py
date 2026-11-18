import numpy as np
import solara
from matplotlib.figure import Figure   # NOT pyplot: pyplot leaks figures in a server
from mesa.visualization import SolaraViz, make_plot_component, make_space_component
from mesa.visualization.utils import update_counter

import driver
import model

def agent_portrayal(agent):
    return AgentPortrayalStyle(
        color="tab:blue",
        size=20,
    )

model_params = {
    "seed": {"type": "InputText", "value": 42, "label": "Seed"},
    "width": 100,
    "height": 100,
    "num_lanes": {"type": "SliderInt", "value": 1, "min": 1, "max": 5, "step": 1, "label": "Num Lanes"},
    "nun_drivers": {"type": "SlinderInt", "value": 1, "min": 1, "max":100, "step":1, "label": "Num Drivers"},
    "t_step": {"type": "SliderInt", "value": 1, "min": 1, "max": 5, "step": 1, "label": "Time Step"}#,
    #Unsure what Agents Param will be/do in TrafficModel
    #"Agents": {"type": "SliderInt", "value": 50, "min": 10, "max": 200, "step": 10, "label": "Agents"},
}

model = TrafficModel(width=100, height=100, num_lanes=1, num_drivers=1, t_step=1, Agents)

# ---------------------------------------------------------------- charts from the DataCollector
SpaceView = make_space_component(agent_portrayal)
SpeedPlot = make_plot_component("mean_speed")  # needs a DataCollector reporter in TrafficModel

page = SolaraViz(
    model,
    components=[SpaceView, SpeedPlot],
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