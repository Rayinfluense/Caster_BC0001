#Version 220728
from student_AutomaticGearbox import student_AutomaticGearbox
import sys

#Interface between matlab_DragsterAssignment and a python version of student_AutomaticGearbox.

if __name__ == "__main__":
    arguments = sys.argv
    result = student_AutomaticGearbox(int(arguments[1]), *[float(i) for i in arguments[2:]])
    gear_demand = result[0]
    throttle = result[1]