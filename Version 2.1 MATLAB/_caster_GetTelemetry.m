function [telemetry, car] = caster_GetTelemetry(car, udpInterfaceTelemetry)
    
    versionnr = 1.4;

    % Sample time
    if(car.dyn.time == 0)
        disp(['GetTelemetry Version ' num2str(versionnr,'%1.1f')])
    end
    if(~isempty(udpInterfaceTelemetry))
        % If udpInterfaceTelemerty isn't empty, then we are online
        telemetry = GetRealTelemetry(udpInterfaceTelemetry);
        car.dyn.rpm = telemetry.RPM;
        car.dyn.acc = telemetry.LongAcc;
        car.dyn.speed = telemetry.Velocity;
        car.dyn.dist = telemetry.Distance;
        car.dyn.time = telemetry.TimeTotal;
        car.dyn.gear = telemetry.Gear;
        car.dyn.throttle = telemetry.Throttle;
        return
    end
    %% Save inputs
    telemetry.Gear = car.dyn.gear;
    telemetry.Throttle = car.dyn.throttle;
    
    %% Vehicle Model
    car = caster_dragsterCar(car);
    telemetry.RPM = car.dyn.rpm;
    telemetry.LongAcc = car.dyn.acc;
    telemetry.Velocity = car.dyn.speed;
    telemetry.Distance = car.dyn.dist;
    telemetry.TimeLap = car.dyn.time;
    telemetry.TimeTotal = car.dyn.time;
    
end

%% Dragster car model
function car = caster_dragsterCar(car)
    
    % Sampling
    sampleTime = 0.01;
    
    % Dynamic paramaters
    rpm = car.dyn.rpm;
    acc = car.dyn.acc;
    speed = car.dyn.speed;
    dist = car.dyn.dist;
    time = car.dyn.time;
    throttle = car.dyn.throttle;
    gear = car.dyn.gear;
    w_wheel = car.dyn.w_wheel;
    
    %% Wheel rotational speed and force
    % Prevent exceeding limits of gear and throttle
    gear = max(min(gear,car.gearbox.gears),-1);
    throttle = max(min(throttle,1),0);
    
    % Derive wheel propulsion force
    if(gear == 0 || gear == -1)
        gear_ratio = 0;
    else
        gear_ratio = car.gearbox.ratio(gear);
    end
    T_engine = interp1(car.engine.rpm, car.engine.torque, rpm,'linear','extrap') * throttle;
    T_wheel = T_engine * gear_ratio * car.gearbox.final / 2;
    F_x_applied = T_wheel / car.wheel.radius;
    
    % Derive wheel peripheral speed
    if(gear_ratio ~= 0)
        w_wheel = (rpm / gear_ratio / car.gearbox.final) * (2*pi) / 60;
    end
    v_wheel = w_wheel * car.wheel.radius;
    
    % Derive wheel slip ratio
    slipratio = (v_wheel / max([speed 0.01]) - 1) * 100;
    
    %% Wheel vertical force
    % Derive aerodynamic drag force
    F_drag = car.aero.drag_coefficient * car.aero.frontal_area * ...
        car.aero.air_density * speed^2 / 2;
         
    % Derive aerodynamic down force
    F_down = car.aero.down_coefficient * car.aero.frontal_area * ...
        car.aero.air_density * speed^2 / 2;
         
    % Derive acceleration force
    F_acc = car.chassis.mass * acc;
    
    % Derive static force
    F_static = car.chassis.mass * 9.81;
    
    % Derive normal load, assuming rear wheel drive, and calculated per wheel
    F_z = (car.chassis.cog_height / car.chassis.wheel_base) * ...
        (F_acc + F_drag) / 2 + ...
        (F_down / 4) + (F_static / 4);
      
    %% Drive force
    NFX = caster_Pacejka96(car.pacejka, F_z, slipratio);
    F_x_capacity = NFX * F_z;
    F_x_drive = min([F_x_capacity, F_x_applied]);
    
    %% Acceleration and Speed
    acc = ((F_x_drive * 2) - F_drag - ...
           (F_static + F_down) * car.wheel.rollcoeff) / ...
            car.chassis.mass;
        
    speed = speed + acc * sampleTime;
    
    %% Excessive force
    F_x_excess = F_x_applied - F_x_capacity;
    
    %% RPM
    % Check RPM as if there was no automatic clutch
    w_wheel_dot = (F_x_excess * car.wheel.radius) / car.wheel.inertia;
    w_wheel = w_wheel + w_wheel_dot * sampleTime;
    rpm_unclutched = w_wheel / (2*pi) * 60 * car.gearbox.final * ...
                                 gear_ratio;
%     w_wheel_grip = speed / car.wheel.radius;
%     w_wheel_slip = w_wheel - w_wheel_grip;
    
    % Automatic clutch to simulate never going below idle rpm
%     clutch = 1 - min([max([0, rpm_unclutched]), car.engine.idle_rpm]) / car.engine.idle_rpm;
    rpm = max([rpm_unclutched, car.engine.idle_rpm]);
    
    %% Distance and Time
    dist = dist + speed * sampleTime;
    time = time + sampleTime;
    
    car.dyn.rpm = rpm;
    car.dyn.acc = acc;
    car.dyn.speed = speed;
    car.dyn.dist = dist;
    car.dyn.time = time;
    car.dyn.throttle = throttle;
    car.dyn.gear = gear;
    car.dyn.w_wheel = w_wheel;
    
end

%% Tire model
function NFX = caster_Pacejka96(pacejka, F_z, slipratio)

    %   © CASTER / Eric Gunnarsson
    %   Created: 2016-05-18
    % Inputs:
    %   pacejka - Pacejka 94 coefficients [N]
    %   Fz - Normal load [N]
    %   slipratio - Slip ratio [%]
    %   
    % Outputs:
    %   NFX - Longitudinal friction [-]
    %

    %% Unit conversions
    F_z = F_z / 1e3;
    % slipratio = slipratio*100;

    %% Pacejka-96
    % Based on: http://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
    % and http://www.racer.nl/docs/projects/lib/rpacejka.htm
    %
    % If website disapears check Giancarlo Genta Motor vehicle dynamics
    % Modeling and simulation Vol43 Page 60-75.
    %
    % Note:
    % - F in [N]
    % - Fz in [kN]
    % - slip [%]
    %--------------

    %% -------------------- Longitudinal----------------------------
    C_x = pacejka.b0;                                         % Shape factor (determines the shape of the peak)
    D_x = (pacejka.b1*F_z+pacejka.b2).*F_z;                              % Peak value of the curve
    BCD_x = (pacejka.b3*F_z.^2 + pacejka.b4*F_z).*exp(-pacejka.b5*F_z);            % Stiffness
    B_x = BCD_x./(C_x*D_x);                              % Stiffnes factor
    Sh_x = pacejka.b9*F_z+pacejka.b10;                                  % Shifting the curve horizontally
    Sv_x = 0;                                           % Shifting the curve vertically
    E_x = (pacejka.b6*F_z.^2+pacejka.b7*F_z+pacejka.b8);                         % Curvature factor

    Fx0=D_x.*sin(C_x.*atan(B_x.*(1-E_x).*(slipratio+Sh_x)+E_x.*atan(B_x.*(slipratio+Sh_x))))+Sv_x;

    %% Normalization
    NFX=Fx0./(F_z*10^3);

end

function Data = GetRealTelemetry(udpInterfaceTelemetry)
    
    % Raw network data
    Raw=[];
    fopen(udpInterfaceTelemetry);
    while numel(Raw) ~= 73
        Raw = fread(udpInterfaceTelemetry, 173, 'single');
        if numel(Raw) == 73
            
            % Assign to output struct
            Data.Gear = double(typecast(single(Raw(3)),'int32'))-1;
            Data.RPM = Raw(4);
            Data.Shiftlight = Raw(5);
            Data.Velocity = Raw(6);
            Data.LatAcc = Raw(10);
            Data.VertAcc = Raw(11);
            Data.LongAcc = Raw(12);
            Data.Pos_x = Raw(13);
            Data.Altitude = Raw(14);
            Data.Pos_y = Raw(15) * -1;
            Data.DistanceSplineLap = Raw(21);
            Data.DistanceSplineTotal = Raw(22);
            Data.DistanceCarLap = Raw(23);
            Data.DistanceCarTotal = Raw(24);
            Data.Lap = double(typecast(single(Raw(25)),'int32'));
            Data.TimeTotal = double(typecast(single(Raw(27)),'int32'))/1000;
            Data.TimeLap = double(typecast(single(Raw(28)),'int32'))/1000;
            Data.Steering = Raw(51);
            Data.Throttle = Raw(52);
            Data.Brake = Raw(53);
            Data.Track = vertcat([char(typecast(single(Raw(66)),'uint8'))...
                                char(typecast(single(Raw(67)),'uint8'))...
                                char(typecast(single(Raw(68)),'uint8'))...
                                char(typecast(single(Raw(69)),'uint8'))...
                                char(typecast(single(Raw(70)),'uint8'))...
                                char(typecast(single(Raw(71)),'uint8'))...
                                char(typecast(single(Raw(72)),'uint8'))...
                                char(typecast(single(Raw(73)),'uint8'))]);
            Data.Car = vertcat([char(typecast(single(Raw(58)),'uint8'))...
                                char(typecast(single(Raw(59)),'uint8'))...
                                char(typecast(single(Raw(60)),'uint8'))...
                                char(typecast(single(Raw(61)),'uint8'))...
                                char(typecast(single(Raw(62)),'uint8'))...
                                char(typecast(single(Raw(63)),'uint8'))...
                                char(typecast(single(Raw(64)),'uint8'))...
                                char(typecast(single(Raw(65)),'uint8'))]);
                            
            Data.Distance=Data.DistanceSplineLap;
        end
        
    end
    fclose(udpInterfaceTelemetry);
    
end