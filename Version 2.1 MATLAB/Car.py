import numpy as np

class Car(object):

    def __init__(self):
        class Dyn(object):
            # Dynamical parameters of car
            def __init__(self, car):
                self.rpm = car.engine.idle_rpm
                self.speed = 0
                self.acc = 0
                self.dist = 0
                self.time = 0
                self.w_wheel = 0
                self.prevshifttime = -1
                self.gotogear = 1
                self.gear = 0
                self.throttle = 0

        class Pacejka():
            #Tires
            def __init__(self):
                self.b0 = 1.6
                self.b1 = -58
                self.b2 = 1750
                self.b3 = 6.8
                self.b4 = 200
                self.b5 = 0
                self.b6 = 0.0034
                self.b7 = -0.008
                self.b8 = -0.76
                self.b9 = 0
                self.b10 = 0
                self.b11 = 0
                self.b12 = 0

        class Engine():
            def __init__(self):
                self.rpm = np.array([0, 511.11, 1211.11, 2044.45,
                                    4166.66, 5200, 5533.34, 5922.22,
                                    6322.22, 6500, 6955.56, 7488.89,
                                    7500, 7922.22, 8000, 8300,
                                    8500, 10000, 10500])

                self.torque = np.array([0.1894, 0.2030, 0.2421, 0.2740,
                                       0.3784, 0.8142, 0.8702, 0.9501,
                                       0.9994, 0.9994, 0.9450, 0.9025,
                                       0.9242, 0.8668, 0.9027, 0.7887,
                                       0.8597, 0, 0])

                self.torque = np.multiply(self.torque, 864)
                self.max_rpm = 8500
                self.idle_rpm = 1850

        class Gearbox():
            def __init__(self):
                self.gears = 4 - 1
                self.ratio = np.array([3.8, 2.3, 1.4, 0.95])
                self.final = 4.03
                self.shiftingtime = 0.1

        class Wheel():
            def __init__(self):
                self.inertia = 2.04 + 2.05 / 2 + 0.1 / 2 + 0.05  # Wheel, engine, gearbox, driveshaft inertia
                self.radius = 0.32
                self.rollcoeff = 0.015

        class Chassis():
            def __init__(self):
                self.mass = 1030 + 46 + 36 + 30
                self.cog_height = 0.4
                self.wheel_base = 2.273

        class Aero():
            def __init__(self):
                self.frontal_area = 1.9
                self.down_coefficient = 0.1
                self.drag_coefficient = 0.3
                self.air_density = 1.225

        # Tires
        self.pacejka = Pacejka()

        # Engine
        self.engine = Engine()

        # Gearbox
        self.gearbox = Gearbox()

        # Wheels
        self.wheel = Wheel()

        # Chassis
        self.chassis = Chassis()

        # Aero
        self.aero = Aero()

        #Dynamical variables
        self.dyn = Dyn(self)