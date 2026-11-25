from mesa import Agent, Model
from mesa.experimental.continuous_space import(ContinuousSpace, ContinuousSpaceAgent)
import random
import driver
import numpy as np
import math
from mesa.datacollection import DataCollector
from collections import deque

LANE_WIDTH = 3.6576  # meters, ~12 ft

class TrafficModel(Model):
    """A traffic model with some number of agents given in the init."""
    def __init__(self, 
                 length=1000, 
                 num_lanes=1,
                 num_drivers=1, 
                 t_step=0.1, 
                 rng=None,
                 #Agents):
        ):
        if rng is None:
            rng = int(np.random.SeedSequence().entropy % (2**32))
        self.seed_used = rng
        super().__init__(rng=rng)

        # Note I can seed the RNG in continuous space to make the models more reproducible.
        self.n_lanes = num_lanes
        self.n_drivers = num_drivers
        self.time_step = t_step
        self.length = length
        self.width = num_lanes * LANE_WIDTH #add a .5 meter on each side? +1?
        self.sim_time = 0.0

        # Note I can seed the RNG in continuous space to make the models more reproducible. 
        self.space = ContinuousSpace(
            dimensions=[[0, self.length], [0, self.width]],
            torus = True,
            random = self.rng,
            n_agents = num_drivers
        )

        car_len, car_w, min_gap = 4.5, 1.8, 2.0
        per_lane = math.ceil(num_drivers / num_lanes)
        spacing = length / per_lane
        min_spacing = car_len + min_gap
        if spacing < min_spacing:
            raise ValueError(f"Too many drivers: {per_lane}/lane needs >= {per_lane*min_spacing:.0f} m of road")

        behaviors = [driver.DANGEROUS, driver.AVERAGE, driver.CAUTIOUS]
        for i in range(num_drivers):
            lane, slot = i % num_lanes, i // num_lanes
            jitter = self.rng.uniform(-1, 1) * (spacing - min_spacing) / 4
            x = (slot * spacing + jitter) % length
            y = (lane + 0.5) * LANE_WIDTH
            b = self.random.choice(behaviors)
            driver.Driver(self, behavior_type=b,
                        init_speed=min(20.0, b.desired_speed),
                        init_lane_num=lane, travel_direction=1,
                        init_pos=np.array([x, y]), length=car_len, width=car_w)

        #seed agents onto road using the above random positions
        # driver.Driver.create_agents(
        #     self,
        #     init_speed = 80.4672, #50 mph = 80.4672 kmh
        #     init_pos = used_positions,
        #     travel_direction = 1, #always right? use 1 for right and 0 for left?
        #     length = 4.5, #length and width of cars right?
        #     width = 1.8, 
        #     init_accel = 0,
        #     behavior=self.random.choices(
        #         [driver.DANGEROUS, driver.AVERAGE, driver.CAUTIOUS],
        #         k=num_drivers),
        #     )

        self.datacollector = DataCollector(
            model_reporters={
                "n_vehicles": lambda m: len(m.agents),
                "mean_speed": lambda m: float(np.mean([a.velo for a in m.agents])),
            },
            agent_reporters={
                "x": lambda a: float(a.position[0]),
                "lane": lambda a: int(a.position[1] // LANE_WIDTH),
                "speed": "velo",
            },
        )
        self.history = deque(maxlen=1500)   # (time, x positions, speeds)
        self._record_history()
        self.datacollector.collect(self) #collect data after init for initial positions

    def _record_history(self):
        xs = np.array([a.position[0] for a in self.agents])
        vs = np.array([a.velo for a in self.agents])
        self.history.append((self.sim_time, xs, vs))

    #MINIMUM SCOPE TEST
    def step(self):
        self.agents.do("update_state")   # phase 1: everyone perceives + computes accel
        self.agents.do("move")           # phase 2: everyone integrates
        self.sim_time += self.time_step
        self._record_history()
        self.datacollector.collect(self)
        
    # Continuous spaces give us a lot of functions for free -- calculate distances, add, remove agents, difference vector, get agents in radius, etc. Look at documention:
    # https://mesa.readthedocs.io/stable/_modules/experimental/continuous_space/continuous_space.html