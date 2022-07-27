from Car import Car
from caster_AutomaticGearbox import caster_AutomaticGearbox

def student_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap):
    car = Car() #Contains all data about the car

    final_drive_ratio = car.gearbox.final
    gear_ratios = car.gearbox.ratio #List of ratios for gears 1 through 4
    rpm_vector = car.engine.rpm #List of rpm values
    torque_vector = car.engine.torque #List of torque values. torque_vector[i] gives maximum torque at rpm_vector[i] rpm
    wheel_radius = car.wheel.radius

    #More data can be obtained from the car object similarly.

    # Logic used for gear shifting
    speed = Velocity * 3.6 #m/s to km/h
    throttle = 1 #Full throttle

    if speed < 30:
        gear_demand = 1
    else:
        gear_demand = 4

    #For comparison, uncomment the line below to see how Caster gearbox performs
    gear_demand = caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)
    return (gear_demand, throttle)