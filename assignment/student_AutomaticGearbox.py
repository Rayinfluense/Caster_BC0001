from caster_AutomaticGearbox import caster_AutomaticGearbox

def student_AutomaticGearbox(gear, RPM, longAcc, velocity, throttle, distance, timeLap):

    #Convert m/s to km/h
    speed = velocity * 3.6
    throttle = 1 #Full throttle

    # Check speed and determine gear. Change this and improve the gearbox
    if speed < 30:
        gear_demand = 1 # First gear
    else:
        gear_demand = 4 # Fourth gear

    # For comparison, uncomment the line below to see how Caster gearbox performs
    #gear_demand = caster_AutomaticGearbox(gear, RPM, longAcc, velocity, throttle, distance, timeLap)

    #Return the selected gear
    return (gear_demand, throttle)