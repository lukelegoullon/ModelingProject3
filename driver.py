from mesa import Agent, Model
from mesa.experimental.continuous_space import(ContinuousSpace, ContinuousSpaceAgent)
from dataclasses import dataclass
import numpy as np
import math



# Change this value -- we may want to go to SI units m/s 
SPEED_LIMIT = 60*0.44

DUMMY_BRAKE = -1
# Rule of thumb for a safe_tailgating distance is 3 seconds of distance
# Update this, shuld change based on speed
# SAFE_TAILGATE = 3 * SPEED_LIMIT

## Braking class
class Braking: 
    def __init__(self, init_time):
        self.is_braking = False

        self.curr_braking = 0
        # Once a decision to brake is made, we start tracking braking times and corresponding brake amounts.
        self.react_times = []
        self.brake_amts = []
        self.act_times = []

        # The time where the braking was initialized
        self.init_time = init_time

# Data class is a simplified class meant to hold data -- automatically generates an init()
# Current parameters to define driving 
@dataclass
class Behavior:
    name: str
    desired_speed: float
    dist_threshold: float
    reaction_time_max: float
    reaction_time_min: float
    ttc_crit: float
    ttc_sensitivity: float
    tailgating_dist_coeff: float
    pass_aggro_score: float
    max_accel: float
    max_brake: float
    brake_coeff: float

# Average driving reaction time in real-world conditions is about 1-1.5 seconds, with median around 0.6 for pure natural reflexes and 1-1.5 when taking into account perception + decision + motor
# FUTURE WORK: I bet that reaction time changes based on the threat, like if the person slams on their breaks right in front of you, reaction time decreases from normal conditions. 
# I think we can have a more sophisticated model of reaction time -- for now I will arbitrarily put values

AVERAGE = Behavior(
    "AVERAGE",
    SPEED_LIMIT + 5, #DO WE NEED TO FIX THIS FOR SI UNITS?
    dist_threshold=6, #CHANGE
    reaction_time_max=1.2,
    reaction_time_min=0.5,
    ttc_crit=4.0,
    ttc_sensitivity=1.0,
    tailgating_dist_coeff=3,
    pass_aggro_score=50,
    max_accel = 4, #CHANGE
    max_brake=8, #CHANGE
    brake_coeff=4 #CHANGE
)

CAUTIOUS = Behavior(
    "CAUTIOUS",
    SPEED_LIMIT,
    dist_threshold=12, #CHANGE
    reaction_time_max=1.0,
    reaction_time_min=0.4,
    ttc_crit=6.0,
    ttc_sensitivity=1.2,
    tailgating_dist_coeff=1.5*3,
    pass_aggro_score=20,
    max_accel = 4, #CHANGE
    max_brake=8, #CHANGE
    brake_coeff=6 #CHANGE
)

DANGEROUS = Behavior(
    "DANGEROUS",
    SPEED_LIMIT + 15,
    dist_threshold=2, #CHANGE
    reaction_time_max=1.5,
    reaction_time_min=0.7,
    ttc_crit=2.0,
    ttc_sensitivity=0.8,
    tailgating_dist_coeff=0.4*3,
    pass_aggro_score=90,
    max_accel = 4, #CHANGE
    max_brake=8, #CHANGE
    brake_coeff=2 #CHANGE
)



# Do the modulus to update the taurus after all collision, ttc checks and everything
class Driver(ContinuousSpaceAgent):
    def __init__(self,model, behavior_type,init_speed, init_lane_num, travel_direction, init_pos, length, width, init_accel = 0):
        super().__init__(model.space, model)
        self.velo = init_speed #* travel_direction
        self.travel_dir = travel_direction
        self.lane = init_lane_num
        self.accel = init_accel

        self.behavior = behavior_type

        # Center of vehicle in global coordinates
        self.position = init_pos

        # Vechiles are represented by rectangles with length and width. 
        self.length = length
        self.width = width

        # Reaction time computation
        # Calculate TTC and gap between leading vehicle *****need to figure out how to determine leading vehicle

# COMMENTED FOR MINIMUM SCOPE TEST
        # self.gap, _,self.ttc = self.compute_ttc(lead_vehicle) # !! Does not run yet !!#
        #self.reaction_time = self.get_reaction_time(self.ttc, self.gap)
        self.braking = None

        # Compute acceleration
        #self.accel = self.compute_acceleration(self.ttc, self.gap, self.reaction_time)

# COMMENTED FOR MINIMUM SCOPE TEST
        # self.inv_kinematics(self.length/2, self.width/2)
        self.update_bumpers()
        self.back_left = self.pos + self.travel_dir

    @staticmethod
    def mph_to_ms(mph_speed):
        ms = 0.44704 * mph_speed
        return ms

    def update_bumpers(self):

        self.front_right = self.pos + [self.travel_dir*self.length / 2, self.width/2]
        self.front_left = self.pos + [self.travel_dir*self.length/2, -self.width/2]

        self.back_right = self.pos + [-self.travel_dir*self.length/2, self.width/2]
        self.back_left = self.pos + [-self.travel_dir*self.length/2, -self.width/2]

    # Get from a point in the driver frame to the frame
    # def inv_kinematics(self,x_loc, y_loc):
    #     yaw = self.yaw
    #     x,y = self.posistion
    #     rot_mat = np.array([[math.cos(yaw), -math.sin(yaw), x],[math.sin(yaw), math.cos(yaw) ,y], [0,0,1]])
    #     point = rot_mat @ np.array([x_loc, y_loc, 1])
    #     return point[:2]

    
    # def set_speed(self, new_speed):
    #     self.speed = new_speed

    # Every driver every time step runs this method. It computes acceleration, checks collision, decides to change lanes, etc.
    def update_state():
        return

    #for minimum scope can change later
    def step(self):
        dt = self.model.time_step
 
        # No lead vehicle yet (1-car test)
        gap, _, ttc = self.compute_ttc(None)
 
        t_react = self.get_reaction_time(ttc, gap)
        self.accel = self.compute_acceleration(ttc, gap, t_react)
 
        # Forward Euler
        self.velo = max(0.0, self.velo + dt * self.accel)
        new_pos = self.position + dt * np.array([self.travel_dir * self.velo, 0.0])
 
        # Ring road: wrap around
        self.position = new_pos % np.array([self.model.length, self.model.width])
 

     # Call this method each time step ONLY if we need decide to brake
    # Inputs:
    # model_time -- in the time stepping function pass model.time here
    # dt -- length of each time step
    # ttc - time to collision, helps get reaction time and brake amount
    def get_brake_amount(self, model_time,dt, ttc, gap):

        brake_amount = 0
        # If we just registered that we need to brake, we have to create a new braking object.  
        if self.braking == None:

            # Here model_time will be the initial brake time
            self.braking = Braking(model_time)
            # We will not have reacted by this point, so return 0
            return 0

        # Get current react time and brake amount based on the perceived threat in this time step. 
        react_time = self.get_reaction_time(ttc)
        brake_amount = self.compute_acceleration( ttc, gap, react_time)

        # 
        self.braking.brake_amts.append(brake_amount)
        self.braking.react_times.append(react_time)
        self.braking.act_times.append(self.braking.init_time + dt*react_time)

        # Get the min activation time and its index -- this will be used to decide if we have reacted by the time step in order to brake
        min_act = min(self.braking.act_times)

        # activation time index
        i = self.braking.act_times.index(min(self.braking.act_times))

        #if we have reached a time to react
        if model_time >= self.braking.act_times[i]:
            self.braking.curr_braking = self.braking.brake_amts[i]
            self.braking.act_times.pop(i)
            self.braking.brake_amts.pop(i)
            self.braking.react_times.pop(i)

        return self.braking.curr_braking

    # Forward Euler Integration + Kinematic bicycle model, as found here: https://github.com/winstxnhdw/KinematicBicycleModel/blob/main/README.md
    def move(self, dt):
        self.position = self.position + dt*np.array([self.velo,0])
        self.velo = self.velo + dt*self.accel


        
    # https://www.geeksforgeeks.org/dsa/find-two-rectangles-overlap/
    # Assume the two cars are travelling the same direction.
    # AI generated function here -- to handle torus
    def check_collision(self, other):
        L = self.model.space.width  # Or your stored road length

        dx = other.pos[0] - self.pos[0]
        dx = (dx + L/2) % L - L/2

        dy = other.pos[1] - self.pos[1]

        x_overlap = abs(dx) <= (self.length + other.length) / 2
        y_overlap = abs(dy) <= (self.width + other.width) / 2

        return x_overlap and y_overlap
    # def check_collision(self, other):

    #     if self.pos[0] < other.pos[0]:
    #     if self.travel_dir > 0:
    #         x1_min = self.back_right[0]
    #         x1_max = self.front_right[0]

    #         x2_min = other.back_right[0]
    #         x2_max = other.front_right[0]

    #     else: 
    #         x1_min = self.front_right[0]
    #         x1_max = self.back_right[0]

    #         x2_min = other.front_right[0]
    #         x2_max = other.back_right[0]

    #     if x1_min > x2_max or x2_min > x1_max:
    #         return False

    #     y1_min = min(self.front_right[1], self.front_left[1])
    #     y2_min = min(other.front_right[1], other.front_left[1])

    #     y1_max = max(self.front_right[1], self.front_left[1])
    #     y2_max = max(other.front_right[1], other.front_left[1])
        

    #     if y1_min > y2_max or y2_min > y1_max:
    #         return False

    #     return True
        

    def colliding(self):
        colliding_vehicles = []
        neighbors,distances = self.get_neighbors_in_radius(15)
        other_drivers = [driver for driver in neighbors if driver is not self]
        
        for driver in other_drivers:
            if(self.check_collision(driver)):
                colliding_vehicles.append(driver)
        return colliding_vehicles

    # Added definitions for controlling reaction time - TTC calculation
    def compute_ttc(self, lead_vehicle):
        """Returns (gap, closing_speed, ttc). Pass None if there is no lead."""
        if lead_vehicle is None:
            return np.inf, 0.0, np.inf
 
        gap = (lead_vehicle.position[0] - self.position[0]
               - 0.5 * self.length - 0.5 * lead_vehicle.length)
        closing_speed = self.velo - lead_vehicle.velo
 
        if closing_speed <= 0:
            return gap, closing_speed, np.inf
        return gap, closing_speed, gap / closing_speed

    
    # Assign reaction time to driver based on TTC
    def get_reaction_time(self, ttc, gap):

        b = self.behavior

        if np.isinf(ttc):
            return b.reaction_time_max

        if gap < (SPEED_LIMIT*b.tailgating_dist_coeff)/4:
            return b.reaction_time_min

        reaction_time = (b.reaction_time_min + (b.reaction_time_max - b.reaction_time_min)
            / (1 + np.exp(-b.ttc_sensitivity * (ttc - b.ttc_crit))))
        
        return reaction_time

    # Formula for calculating acceleration/deceleration
    def compute_acceleration(self, ttc, gap, t_react):
        b = self.behavior
        a_free = (1 - self.velo/b.desired_speed) * b.max_accel

        if gap > b.dist_threshold:
            return a_free
        
        a_brake = b.max_brake * (1 / (1 + np.exp(b.brake_coeff * (ttc - t_react))))
        return (a_free - a_brake)


if __name__ == "__main__":
    print('hi')
