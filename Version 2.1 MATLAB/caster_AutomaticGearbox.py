#Version 220728
from Car import Car
import numpy as np
import math

def caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap):
    ##CASTER / Eric Matsson, Python version: Samuel Sandelius
    #CREATED: 2016-06-05, Python version: 2022-06-21

    #Calculate the "optimal" gear at any given time give nthe telemtry data from the
    #vehicle.

    #Utilizes engine torque-map and gear aratios to calculate shifting points and
    #compare sthis to the current state of the vehicle in order to select the
    #"best" gear.

    versionnr = 1.3

    car = Car() #Contains all data about the car

    num_gears = len(car.gearbox.ratio)

    if True: #In case we want to save the results to prevent doing the calculations multiple times
        final_drive_ratio = car.gearbox.final
        gear_ratios = np.concatenate(([0], car.gearbox.ratio)) #List of ratios for gears 1 through 4
        rpm_vector = car.engine.rpm #List of rpm values
        torque_vector = car.engine.torque #List of torque values. torque_vector[i] gives maximum torque at rpm_vector[i] rpm
        wheel_radius = car.wheel.radius

        #print("caster_AutomaticGearbox Version ", versionnr) #Can I make this only print once?

        rpm_resolution = 1

        speed_ratios = np.zeros(num_gears)
        shift_up_RPMs = np.zeros(num_gears-1)
        shift_down_RPMs = np.zeros(num_gears)

        power_factor = np.multiply(rpm_vector, torque_vector/(30 * math.pi * 1000))

        for i in range(1, num_gears):
            speed_ratios[i] = gear_ratios[i+1] / gear_ratios[i]

        #The rpm-torque relation will be compared for different gears at the rpms given in rpm_vector
        rpms = list(range(round(max(rpm_vector) / 2) , round(max(rpm_vector)), rpm_resolution))

        #rpms where two different gears generate the same power
        for i in range(1, len(speed_ratios)):
            low_gear = np.interp(rpms, rpm_vector, power_factor)
            high_gear = np.interp(np.multiply(rpms , speed_ratios[i]), rpm_vector, power_factor)
            shift_up_RPMs[i-1] = rpms[np.where(high_gear > low_gear)[0][0]]
            shift_down_RPMs[i] = rpms[np.where(high_gear > low_gear)[0][0]] * speed_ratios[i]

        #Calculate which velocities correspond to the calculated shifting rpms
        shift_up_velocities = np.divide(np.divide(shift_up_RPMs, gear_ratios[range(1, len(gear_ratios) - 1)]) , final_drive_ratio * 60 / (wheel_radius * 2 * math.pi))
        shift_down_velocities = np.divide(np.divide(shift_down_RPMs, gear_ratios[range(1, len(gear_ratios))]) , final_drive_ratio * 60 / (wheel_radius * 2 * math.pi * 0.95))

    #Get current gear
    gear_demand = Gear
    if gear_demand > 0:
        #Check if the velocity is higher/(lower than the shifting velocity for the current gear and if that is the
        #case shift the gear one step in the desired direction.
        if Gear < num_gears and Velocity > shift_up_velocities[Gear - 1]:
            gear_demand = Gear + 1
        elif Gear > 1.1 and Velocity < shift_down_velocities[Gear - 1]:
            gear_demand = Gear - 1
    #To make sure the simulator doe snot get stuck in neutral/revers
    else:
        gear_demand = 4
    return gear_demand
