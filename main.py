from driver import Driver, Behavior, Braking, AVERAGE, DANGEROUS, CAUTIOUS
from model import TrafficModel

def main():

    model = TrafficModel(1000, 1, 2, 0.1)
    d1 = Driver(model, AVERAGE, Driver.mph_to_ms(30), 1, 1, [999.5,0], 2.5, 1.6)
    d2 = Driver(model, AVERAGE, Driver.mph_to_ms(30), 1, 1, [3,0], 2.5, 1.6)
    
    colliding = d1.check_collision(d2)
    # Should evaluate to True
    print("Driver 1 is colliding with Driver 2: ", colliding)

main()