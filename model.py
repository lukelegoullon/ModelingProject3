from mesa import Agent, Model
from mesa.experimental.continuous_space import(ContinuousSpace, ContinuousSpaceAgent)
import random
import driver
from mesa.datacollection import DataCollector

class TrafficModel(Model):

    def __init__(self, width, height, num_lanes, num_drivers, t_step, Agents):
        super().__init__()

        # Note I can seed the RNG in continuous space to make the models more reproducible. 
        self.space = ContinuousSpace(dimension = [width, height],torus = True, n_agents = len(Agents))
        self.n_lanes = num_lanes
        self.type = "Torus"
        self.n_drivers = num_drivers
        self.time_step = t_step

        self.speed_hist: list[np.ndarray] = []  # per step, speed per (lane, cell) for space-time plots

        self.datacollector = DataCollector(
            model_reporters={
                "n_vehicles": lambda m: len(m.agents),
                "mean_speed": lambda m: np.mean([a.v for a in m.agents]) if len(m.agents) else 0.0,
                "throughput": lambda m: m.throughput,
                **{
                    f"queue_{e['name']}": (lambda m, name=e["name"]: m.queue_of(name))
                    for e in self.entries
                },
            }
        )
        # At end of simulation, need a time-snapshot to record speed and position for each car
        #for agent in agents:
            #snap = agent(pos,vel)
            #self.speed_hist.append(snap)
        #at end collect the information in the datacollector component inside the model
        #self.datacollector.collect(self)
        

        

    def get_drivers(self):
        drivers = []
        for i in range(self.n_drivers):
            driver_behavior = random.choice([driver.DANGEROUS, driver.AVERAGE, driver.CAUTIOUS])

            new_driver = driver.Driver(TrafficModel, 40, 1, 1, )

    # Continuous spaces give us a lot of functions for free -- calculate distances, add, remove agents, difference vector, get agents in radius, etc. Look at documention:
    # https://mesa.readthedocs.io/stable/_modules/experimental/continuous_space/continuous_space.html