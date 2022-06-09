import copy
import matplotlib.pyplot as plt
from Car import Car
import math
import numpy as np
from student_AutomaticGearbox import student_AutomaticGearbox

def caster_GearShift(gear_demand, prevshifttime, car, plotting):
    versionnr = 1.3

    if prevshifttime == 0 and plotting:
        print("GearShift version ", versionnr)

    mintimeShift = car.gearbox.shiftingtime

    #Limit to allowed gears
    gear_demand = min(gear_demand, car.gearbox.gears)
    gear_demand = max(-1, gear_demand)

    #Check so that the desired shift time has passed
    if car.dyn.time - prevshifttime >= mintimeShift:
        #Update gear in the model
        if gear_demand != car.dyn.gear:
            #Make an arbitrary change to neutral to simulate clutch
            if car.dyn.gear != -1:
                car.dyn.gear = -1
                prevshifttime = car.dyn.time
                car.dyn.gotogear = gear_demand
            else:
                car.dyn.gear = car.dyn.gotogear
                prevshifttime = car.dyn.time

    return prevshifttime

def caster_GetTelemetry(car, plotting):

    class Telemetry(object):
        def __init__(self, car):
            self.Gear = car.dyn.gear
            self.Throttle = car.dyn.throttle
            caster_dragsterCar(car) #Step vehicle model
            self.RPM = car.dyn.rpm
            self.LongAcc = car.dyn.acc
            self.Velocity = car.dyn.speed
            self.Distance = car.dyn.dist
            self.TimeLap = car.dyn.time
            self.TimeTotal = car.dyn.time

    versionnr = 1.4

    if car.dyn.time == 0 and plotting:
        print("GetTelemetry version " , versionnr)

    #Save inputs
    #caster_dragsterCar(car) #step vehicle model
    telemetry = Telemetry(car)
    return telemetry

def caster_dragsterCar(car):
    #Simulates and updates dynamic parameters of car.
    sampleTime = 0.01 #Sampling

    #Dynamic parameters
    rpm = car.dyn.rpm
    acc = car.dyn.acc
    speed = car.dyn.speed
    dist = car.dyn.dist
    time = car.dyn.time
    throttle = car.dyn.throttle
    gear = car.dyn.gear
    w_wheel = car.dyn.w_wheel

    #Wheel rotational speed and force
    #Prevent exceeding limits of gear and throttle
    gear = max(min(gear, car.gearbox.gears), -1)
    throttle = max(min(throttle, 1), 0)

    #Derive wheel propulsion force
    if gear == -1:
        gear_ratio = 0 #In neutral
    else:
        gear_ratio = car.gearbox.ratio[gear]

    T_engine = np.interp(rpm, car.engine.rpm, car.engine.torque) * throttle #Does not extrapolate like Matlab version.
    T_wheel = T_engine * gear_ratio * car.gearbox.final / 2
    F_x_applied = T_wheel / car.wheel.radius

    #Derive wheel peripheral speed
    if gear_ratio != 0:
        w_wheel = (rpm / gear_ratio / car.gearbox.final) * (2 * math.pi) / 60

    v_wheel = w_wheel * car.wheel.radius

    #Derive wheel slip ratio
    slipratio = (v_wheel / max(speed, 0.01) - 1) * 100

    #Wheel vertical force
    #Derive aerodynamic drag force
    F_drag = car.aero.drag_coefficient * car.aero.frontal_area * car.aero.air_density * speed ** 2 / 2

    #Derive aerodynamic down force
    F_down = car.aero.down_coefficient * car.aero.frontal_area * car.aero.air_density * speed ** 2 / 2

    #Derive acceleration force
    F_acc = car.chassis.mass * acc

    #Derive static force
    F_static = car.chassis.mass * 9.81

    #Derive normal load, assuming rear wheel drive, and calculated per wheel
    F_z = (car.chassis.cog_height / car.chassis.wheel_base) * (F_acc + F_drag) / 2 + (F_down / 4) + (F_static / 4)

    #Drive force
    NFX = caster_Pacejka96(car.pacejka, F_z, slipratio)
    F_x_capacity = NFX * F_z
    F_x_drive = min(F_x_capacity, F_x_applied)

    #Acceleration and speed
    acc = ((F_x_drive * 2) - F_drag - (F_static + F_down) * car.wheel.rollcoeff) / car.chassis.mass
    speed = speed + acc * sampleTime

    #Excessive force
    F_x_excess = F_x_applied - F_x_capacity
    #RPM
    #Check RPM as if there was no automatic clutch
    w_wheel_dot = (F_x_excess * car.wheel.radius) / car.wheel.inertia
    w_wheel = w_wheel + w_wheel_dot * sampleTime

    rpm_unclutched = w_wheel / (2 * math.pi) * 60 * car.gearbox.final * gear_ratio

    rpm = max(rpm_unclutched, car.engine.idle_rpm)

    #Distance and time
    dist = dist + speed * sampleTime
    time = time + sampleTime

    car.dyn.rpm = rpm
    car.dyn.acc = acc
    car.dyn.speed = speed
    car.dyn.dist = dist
    car.dyn.time = time
    car.dyn.throttle = throttle
    car.dyn.gear = gear
    car.dyn.w_wheel = w_wheel

def caster_Pacejka96(pacejka, F_z, slipratio):

    F_z = F_z / 1000
    C_x = pacejka.b0
    D_x = (pacejka.b1*F_z + pacejka.b2) * F_z
    BCD_x = (pacejka.b3 * F_z ** 2 + pacejka.b4 * F_z) * math.exp(-pacejka.b5 * F_z)
    B_x = BCD_x / (C_x*D_x)
    Sh_x = pacejka.b9 * F_z + pacejka.b10
    Sv_x = 0
    E_x = (pacejka.b6 * F_z ** 2 + pacejka.b7 * F_z + pacejka.b8)
    Fx0=D_x * math.sin(C_x * math.atan(B_x * (1-E_x) * (slipratio + Sh_x) + E_x * math.atan(B_x * (slipratio + Sh_x))))+Sv_x
    NFX = Fx0/(F_z * 1000)

    return NFX

def caster_InitCommunication(plotting):
    versionnr = 1.5

    if plotting:
        print("InitCommunication Version ", versionnr)

    car = Car()
    return car

def main(plotting):
    #CASTER / Håkan Richardsson, Richard Löfwenberg, Jonas Johnsson, Alexander Hägglund
    #Created: 2016-06-05
    #Last edit:

    #Simulates a vehicle at a dragstrip by calling the function
    #student_AutomaticGearbox for gear target and throttle

    #Argument online should only be used in the caster simulator. For
    #offline use, set it to 0. (Online use not available in this, Python version)
    #If plotting is set to 0, no plots will appear at the end of the
    #run and console output will be minimized.

    print("Main program Version py_1.0")

    #Establish communication with car

    car = caster_InitCommunication(plotting)

    dist_goal = 402
    distance = 0
    telemetrylog = list()
    prevshifttime = 1
    initialRun = True

    while(distance <= dist_goal):
        if initialRun: #Ask for gear before a time step is taken
            (gear_demand, throttle) = student_AutomaticGearbox(car.dyn.gear, car.dyn.rpm, car.dyn.acc, car.dyn.speed, car.dyn.throttle, car.dyn.dist, car.dyn.time)
            initialRun = False
        else:
            #Get telemetry
            telemetry = caster_GetTelemetry(car, plotting) #Also steps car in simulation
            telemetrylog.append(copy.deepcopy(telemetry))
            (gear_demand, throttle) = student_AutomaticGearbox(telemetry.Gear, telemetry.RPM, telemetry.LongAcc, telemetry.Velocity, telemetry.Throttle, telemetry.Distance, telemetry.TimeLap)
            distance = telemetry.Distance
        if plotting:
            if gear_demand < 0:
                print("You have tried to put in reverse or neutral (", gear_demand, "). While this works physically, the simulation will probably get stuck in an infinite loop since the car won't reach the finish line")
            if gear_demand > car.gearbox.gears:
                print("You have tried to put in the gear ", gear_demand + 1, ". This car only has ", car.gearbox.gears + 1, "gears. Did you think you could cheat? Picking gear ", car.gearbox.gears + 1, " instead.")
            if throttle > 1:
                print("You have tried to give a throttle higher than 1 (", throttle, "), which would be equal to pressing the pedal through the floor. Since I don''t want holes in the floor, i will change the throttle to 1")
            if throttle < 0:
                print("You have tried to give a throttle lower than 0, which would be equal to ripping the pedal loose. Since I want to keep the pedal I will change the throttle to 0")
            if throttle == 0:
                print("No throttle is being given. This won't move the car and will probably leave the simulation stuck in a loop.")
            if round(gear_demand) != gear_demand:
                print("Gear input must be an integer within the range [1, 4]. The input was " ,gear_demand + 1 ,"." )
        if gear_demand < -1:
            gear_demand = 0
        gear_demand = round(gear_demand)
        car.dyn.throttle = throttle
        prevshifttime = caster_GearShift(gear_demand, prevshifttime, car, plotting) #Also updates gear in car.dyn

    if plotting:
        fig, axs = plt.subplots(2,2)
        distancelog = [telemetry.Distance for telemetry in telemetrylog]

        velocitylog = [telemetry.Velocity for telemetry in telemetrylog]
        rpmlog = [telemetry.RPM for telemetry in telemetrylog]
        gearlog = [telemetry.Gear for telemetry in telemetrylog]
        longacclog = [telemetry.LongAcc for telemetry in telemetrylog]

        axs[0, 0].scatter(distancelog, np.multiply(velocitylog, 3.6), s=1)
        axs[0, 0].set(xlabel = "Distance [m]", ylabel = "Velocity [km/h]")

        axs[0, 1].scatter(distancelog, rpmlog, s=1)
        axs[0, 1].set(xlabel = "Distance [m]", ylabel = "Engine RPM")

        axs[1, 0].scatter(distancelog, np.add(gearlog, 1), s=1)
        axs[1, 0].set_title("")
        axs[1, 0].set(xlabel = "Distance [m]", ylabel = "Current Gear")

        axs[1, 1].scatter(distancelog, longacclog, s=1)
        axs[1, 1].set_title("")
        axs[1, 1].set(xlabel = "Distance [m]", ylabel = "Longitudinal Acceleration [m/s^2]")

        timelaplog = [telemetry.TimeLap for telemetry in telemetrylog]
        finishtime = np.interp(dist_goal, distancelog, timelaplog)
        print("Finish time: " , "{:.2f}".format(finishtime) , " seconds.")

        plt.show()

if __name__ == "__main__":
    main(True)
