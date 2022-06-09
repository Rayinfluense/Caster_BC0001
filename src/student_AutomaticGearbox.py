def student_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap):

    speed = Velocity * 3.6
    throttle = 1

    if speed < 30:
        gear_demand = 1
    else:
        gear_demand = 4

    #For comparison, uncomment the line below to see how Caster gearbox performs
    #gear_demand = caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)

    return (gear_demand - 1, throttle)