function [gear_demand, Throttle] = student_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)
% Convert m/s to km/h
Speed = Velocity * 3.6;
Throttle = 1;
% Check the speed and determine gear
if Speed < 30
    gear_demand = 1;
else 
    gear_demand = 4;
end

% For comparasion, uncomment the line below to see how caster gearbox performs
gear_demand = caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap);
end