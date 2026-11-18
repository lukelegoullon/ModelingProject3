from mesa import Agent, Model
from mesa.experimental.continuous_space import(ContinuousSpace, ContinuousSpaceAgent)
from dataclasses import dataclass
import numpy as np
import math


def mph_to_ms(mph_speed):
    ms = 0.44704 * mph_speed
    return ms


# Change this value -- we may want to go to SI units m/s 
SPEED_LIMIT = mph_to_ms(60)

# Rule of thumb for a safe_tailgating distance is 3 seconds of distance
# Update this, shuld change based on speed
# SAFE_TAILGATE = 3 * SPEED_LIMIT


# Data class is a simplified class meant to hold data -- automatically generates an init()
# Current parameters to define driving 
@dataclass
class Behavior:
    name: str
    desired_speed: float
    reaction_time_max: float
    reaction_time_min: float
    ttc_crit: float
    ttc_sensitivity: float
    tailgating_dist_coeff: float
    pass_aggro_score: float

# Average driving reaction time in real-world conditions is about 1-1.5 seconds, with median around 0.6 for pure natural reflexes and 1-1.5 when taking into account perception + decision + motor
# FUTURE WORK: I bet that reaction time changes based on the threat, like if the person slams on their breaks right in front of you, reaction time decreases from normal conditions. 
# I think we can have a more sophisticated model of reaction time -- for now I will arbitrarily put values

AVERAGE = Behavior(
    "AVERAGE",
    SPEED_LIMIT + 5,
    reaction_time_max=1.2,
    reaction_time_min=0.5,
    ttc_crit=4.0,
    ttc_sensitivity=1.0,
    tailgating_dist_coeff=0.8*3,
    pass_aggro_score=50
)

CAUTIOUS = Behavior(
    "CAUTIOUS",
    SPEED_LIMIT,
    reaction_time_max=1.0,
    reaction_time_min=0.4,
    ttc_crit=6.0,
    ttc_sensitivity=1.2,
    tailgating_dist_coeff=1.5*3,
    pass_aggro_score=20
)

DANGEROUS = Behavior(
    "DANGEROUS",
    SPEED_LIMIT + 15,
    reaction_time_max=1.5,
    reaction_time_min=0.7,
    ttc_crit=2.0,
    ttc_sensitivity=0.8,
    tailgating_dist_coeff=0.4*3,
    pass_aggro_score=90
)



class Driver(ContinuousSpaceAgent):
    def __init__(self,model,init_speed, init_lane_num, travel_direction, init_pos, length, width, init_accel = 0):
        super().__init__(model.space, model)
        self.velo = init_speed * travel_direction
        self.travel_dir = travel_direction
        self.lane = init_lane_num
        self.accel = init_accel
        # self.wheelbase = 0.65*self.length

        # # yaw is angle of travel from positive y axis. 
        # self.yaw = init_yaw

        # Center of vehicle in global coordinates
        self.pos = init_pos

        # Vechiles are represented by rectangles with length and width. 
        self.length = length
        self.width = width

        

        self.front_right = self.inv_kinematics(self.length / 2, -self.width/2)
        self.front_left = self.inv_kinematics(self.length/2, self.width/2)
        self.back_right = self.inv_kinematics(-self.length/2, -self.width/2)
        self.back_left = self.inv_kinematics(-self.length/2, self.width/2)



    # Get from a point in the driver frame to the frame
    def inv_kinematics(self,x_loc, y_loc):
        yaw = self.yaw
        x,y = self.pos
        rot_mat = np.array([[math.cos(yaw), -math.sin(yaw), x],[math.sin(yaw), math.cos(yaw) ,y], [0,0,1]])
        point = rot_mat @ np.array([x_loc, y_loc, 1])
        return point[:2]

    def set_speed(self, new_speed):
        self.speed = new_speed

    # UPDATE yaw, speeds, and bumper locations. 
    def update_state():
        return


    # Forward Euler Integration + Kinematic bicycle model, as found here: https://github.com/winstxnhdw/KinematicBicycleModel/blob/main/README.md
    def move(self, dt):
        self.pos = self.pos + dt*np.array([self.velo,0])
        self.velo = self.velo + dt*self.accel


        

    # check collision with two cars moving in the same direction 
    def check_collision(self, other):
        
        return True
        

    def colliding(self):
        colliding_vehicles = []
        neighbors,distances = self.get_neighbors_in_radius(15)
        other_drivers = [driver for driver in neighbors if driver is not self]
        
        for driver in neighbors:
            if(self.check_collision(driver)):
                colliding_vehicles.append(driver)
        return colliding_vehicles

    # Added definitions for controlling reaction time - TTC calculation
    def compute_ttc(self, lead_vehicle):
        gap = (
        lead_vehicle.pos[0]
        - self.pos[0]
        - 0.5*self.length
        - 0.5*lead_vehicle.length
    )
        closing_speed = self.velo - lead_vehicle.velo

        if closing_speed <= 0:
            return np.inf

        return gap / closing_speed

    # Assign reaction time to driver based on TTC
    def get_reaction_time(self, ttc):

        b = self.behavior

        if np.isinf(ttc):
            return b.reaction_time_max

        return (
            b.reaction_time_min
            + (b.reaction_time_max - b.reaction_time_min)
            / (1 + np.exp(-b.ttc_sensitivity *
                        (ttc - b.ttc_critical)))
        )
