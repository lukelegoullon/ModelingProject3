import numpy as np
import solara
from matplotlib.figure import Figure   # NOT pyplot: pyplot leaks figures in a server
from mesa.visualization import SolaraViz, make_plot_component, make_space_component
from mesa.visualization.utils import update_counter
from mesa.visualization.components import AgentPortrayalStyle

import driver
import model

def agent_portrayal(agent):
    return AgentPortrayalStyle(color="tab:blue", size=20)
 
 
model_params = {
    "seed": 42,
    "length": {"type": "SliderInt", "value": 1000, "min": 100, "max": 5000, "step": 100, "label": "Road length (m)"},
    "num_lanes": {"type": "SliderInt", "value": 1, "min": 1, "max": 5, "step": 1, "label": "Num Lanes"},
    "num_drivers": {"type": "SliderInt", "value": 1, "min": 1, "max": 100, "step": 1, "label": "Num Drivers"},
    "t_step": {"type": "SliderFloat", "value": 0.1, "min": 0.01, "max": 1.0, "step": 0.01, "label": "Time Step (s)"},
}
 
traffic_model = TrafficModel(length=1000, num_lanes=1, num_drivers=1, t_step=0.1, seed=42)
 
SpaceView = make_space_component(agent_portrayal)
SpeedPlot = make_plot_component("mean_speed")
 
page = SolaraViz(
    traffic_model,
    components=[SpaceView, SpeedPlot],
    model_params=model_params,
    name="Ring-road with no ramps",
    play_interval=100,
)
page  # Solara looks for a top-level `page`
 
if __name__ == "__main__":
    import subprocess, sys
    subprocess.run([sys.executable, "-m", "solara", "run", __file__])