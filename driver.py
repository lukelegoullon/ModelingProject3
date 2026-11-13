from mesa import Agent, Model
from mesa.experimental.continuous_space import(ContinuousSpace, ContinuousSpaceAgent)
from dataclass import dataclass



# Change this value -- we may want to go to SI units m/s 
SPEED_LIMIT = 60

# Rule of thumb for a safe_tailgating distance is 3 seconds of distance
SAFE_TAILGATE = 3 * SPEED_LIMIT

# Data class is a simplified class meant to hold data -- automatically generates an init()
# Current parameters to define driving 
@dataclass
class Behavior:
    name: str
    desired_speed: float
    reaction_time: float
    tailgating_dist: float
    pass_aggro_score: float

# Average driving reaction time in real-world conditions is about 1-1.5 seconds, with median around 0.6 for pure natural reflexes and 1-1.5 when taking into account perception + decision + motor
# FUTURE WORK: I bet that reaction time changes based on the threat, like if the person slams on their breaks right in front of you, reaction time decreases from normal conditions. 
# I think we can have a more sophisticated model of reaction time -- for now I will arbitrarily put values
# We
AVERAGE = Behavior(
    "AVERAGE",
    SPEED_LIMIT + 5,
    reaction_time = 1.2,
    tailgating_dist = 0.8*SAFE_TAILGATE,
    pass_aggro_score = 50
)

CAUTIOUS  = Behavior(
    "CAUTIOUS",
    SPEED_LIMIT,
    reaction_time = 0.9,
    tailgating_dist = 1.5*SAFE_TAILGATE,
    pass_aggro_score = 20
    )

DANGEROUS = Behavior(
    "DANGEROUS",
    SPEED_LIMIT + 15,
    reaction_time = 1.5,
    tailgating_dist = 0.4*SAFE_TAILGATE,
    pass_aggro_score = 90
)

class Driver(Agent):
    def __init__(self,model,init_speed, init_lane_num, travel_direction, init_x, init_y, length):
        self.model = model
        self.speed = init_speed
        self.lane = init_lane_num
        self.direction = travel_direction
        self.x = init_x
        self.y = init_y
        self.length = length

    def set_speed(self, new_speed):
        self.speed = new_speed


    

    