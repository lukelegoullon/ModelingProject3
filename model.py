from mesa import Agent, Model
from mesa.experimental.continuous_space import(ContinuousSpace, ContinuousSpaceAgent)
import random
import driver
import numpy as np
from mesa.datacollection import DataCollector

LANE_WIDTH = 3.6576  # meters, ~12 ft

class TrafficModel(Model):
    """A traffic model with some number of agents given in the init."""
    def __init__(self, length=1000, 
                 num_lanes=1,
                 num_drivers=1, 
                 t_step=0.1, 
                 seed=None#,
                 #Agents):
        ):
        
        super().__init__(seed=seed)
        self.n_lanes = num_lanes
        self.n_drivers = num_drivers
        self.time_step = t_step
        self.length = length
        self.width = num_lanes * LANE_WIDTH #add a .5 meter on each side? +1?

        # Note I can seed the RNG in continuous space to make the models more reproducible. 
        self.space = ContinuousSpace(
            dimensions=[[0, self.length], [0, self.width]],
            torus = True,
            random = self.rng,
            n_agents = num_drivers
        )

        #create np.column array of (x,y) positions for each driver
            #need to ensure drivers are not placed on top of eachother

        used_positions = [] #list to hold positions currently occupied
        while len(used_positions) != self.n_drivers:
            x = self.rng.uniform(0, self.length)
            # NEED TO ACCOUNT FOR SIZE OF CARS IN POSITION SEEDING
            while x in used_positions: #generate a unique x position on the road
                x = self.rng.uniform(0, self.length)
            #choose y to be center of lane 0 or higher
            lane = self.rng.integers(0, self.n_lanes)
            y = (lane+.5)*LANE_WIDTH
            while y in used_positions: #generate a unique x position on the road
                lane = self.rng.integers(0, self.n_lanes)
                y = (lane+.5)*LANE_WIDTH
        #used_positions for each driver contains an (intial x, inital lane)

        #seed agents onto road using the above random positions
        driver.Driver.create_agents(
            self,
            init_speed = 80.4672, #50 mph = 80.4672 kmh
            init_pos = used_positions,
            travel_direction = 1, #always right? use 1 for right and 0 for left?
            length = 4.5, #length and width of cars right?
            width = 1.8, 
            init_accel = 0,
            behavior=self.random.choices(
                [driver.DANGEROUS, driver.AVERAGE, driver.CAUTIOUS],
                k=num_drivers),
            )

        self.datacollector = DataCollector(
            model_reporters={
                "n_vehicles": lambda m: len(m.agents),
                "mean_speed": lambda m: float(np.mean([a.v for a in m.agents])),
            },
            agent_reporters={
                "x": lambda a: float(a.position[0]),
                "lane": lambda a: int(a.position[1] // LANE_WIDTH),
                "speed": "v",
            },
        )

        def step(self):
            self.agents.do("calculate_updated_positions")
            self.agents.do("update_positions")
            self.datacollector.collect(self)
        
    # def get_drivers(self):
    #     drivers = []
    #     for i in range(self.n_drivers):
    #         driver_behavior = random.choice([driver.DANGEROUS, driver.AVERAGE, driver.CAUTIOUS])

    #         new_driver = driver.Driver(TrafficModel, 40, 1, 1, )

    # Continuous spaces give us a lot of functions for free -- calculate distances, add, remove agents, difference vector, get agents in radius, etc. Look at documention:
    # https://mesa.readthedocs.io/stable/_modules/experimental/continuous_space/continuous_space.html