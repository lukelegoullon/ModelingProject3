from mesa import Agent, Model
from mesa.experimental.continuous_space import(ContinuousSpace, ContinuousSpaceAgent)
import random
import driver

class TrafficModel(Model):

    def __init__(self, width, height, num_lanes, num_drivers, t_step, Agents):
        super().__init__()

        # Note I can seed the RNG in continuous space to make the models more reproducible. 
        self.space = ContinuousSpace(dimensions = [[0,width],[0, height]],torus = True, n_agents = len(Agents))
        self.n_lanes = num_lanes
        self.type = "Torus"
        self.n_drivers = num_drivers
        self.time_step = t_step

        self.time = 0

    

        

    def get_drivers(self):
        drivers = []
        for i in range(self.n_drivers):
            driver_behavior = random.choice([driver.DANGEROUS, driver.AVERAGE, driver.CAUTIOUS])

            new_driver = driver.Driver(TrafficModel, 40, 1, 1, )

    # Continuous spaces give us a lot of functions for free -- calculate distances, add, remove agents, difference vector, get agents in radius, etc. Look at documention:
    # https://mesa.readthedocs.io/stable/_modules/experimental/continuous_space/continuous_space.html