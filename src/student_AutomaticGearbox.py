from Car import Car
from caster_AutomaticGearbox import caster_AutomaticGearbox

def student_AutomaticGearbox(gear, RPM, longAcc, velocity, throttle, distance, timeLap):

    car = Car() # Contains all data about the car

    # Examples of data that can be obtained from car object. See Car_reference.py for all available parameters.

    final_drive_ratio = car.gearbox.final
    gear_ratios = car.gearbox.ratio #List of ratios for gears 1 through 4. Gear 0 is neutral and outputs 0 torque.
    rpm_vector = car.engine.rpm #List of rpm values
    torque_vector = car.engine.torque #List of torque values. torque_vector[i] returns maximum torque output at rpm_vector[i] rpm
    wheel_radius = car.wheel.radius
    optimal_slip = car.optimal_slip # Percent

    #Convert m/s to km/h
    speed = velocity * 3.6
    throttle = 1 #Full throttle

    # Check speed and determine gear. Change this and improve the gearbox
    if speed < 30:
        gear_demand = 1 # First gear
    else:
        gear_demand = 4 # Fourth gear

    # For comparison, uncomment the line below to see how Caster gearbox performs
    # gear_demand = caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)

    #Return the selected gear
    return (gear_demand, throttle)