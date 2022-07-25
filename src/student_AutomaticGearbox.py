from Car import Car
from caster_AutomaticGearbox import caster_AutomaticGearbox

def student_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap):
    #SHOULD THIS BE A CLASS INSTEAD SO THE STUDENT CAN USE "GLOBAL" VARIABLES RATHER THAN HAVING TO RECALCULATE CONSTANTS
    #EACH TIME THE METHOD IS CALLED? IS IT WITHIN THE SCOPR OF THE ASSIGNMENT?

    car = Car() #Contains all data about the car

    final_drive_ratio = car.gearbox.final
    gear_ratios = car.gearbox.ratio #List of ratios for gears 1 through 4
    rpm_vector = car.engine.rpm #List of rpm values
    torque_vector = car.engine.torque #List of torque values. torque_vector[i] gives maximum torque at rpm_vector[i] rpm
    wheel_radius = car.wheel.radius

    #More data can be obtained from the car object similarly.

    # Logic used for gear shifting
    speed = Velocity * 3.6
    throttle = 1

    if speed < 30:
        gear_demand = 1
    else:
        gear_demand = 4

    #For comparison, uncomment the line below to see how Caster gearbox performs
    gear_demand = caster_AutomaticGearbox(Gear + 1, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)

    return (gear_demand - 1, throttle)