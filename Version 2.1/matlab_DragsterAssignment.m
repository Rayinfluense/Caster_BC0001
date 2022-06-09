function [telemetrylog, finishtime] = matlab_DragsterAssignment(online, plotting)
    %MATLAB_DRAGSTERASSIGNMENT
    %   � CASTER / H�kan Richardsson, Richard L�fwenberg, Jonas Johnsson,
    %   ALexander H�gglund
    %   Created: 2016-06-05
    %   Last edit: 2021-03-04 by Jonas J, Alexander H
    %   telemetrylog = matlab_DragsterAssignment(online, plotting)
    %
    %   Simulates a vehicle at a dragstrip by calling the function
    %   student_AutomaticGearbox for gear target and throttle
    %   
    %   Argument online should only be used in the caster simulator. For
    %   offline use, set it to 0.
    %   If plotting is set to 0, no plots will appear at the end of the
    %   run and console output will be minimized.

    
    disp('Main Program Version 2.0')
    
    % Establish communication with car
    if(nargin == 0)
        online = 0;
        plotting = 1;
    elseif(nargin == 1)
        plotting = 1;
    end
    [car, udpInterfaceTelemetry, udpInterfaceConsole] = caster_InitCommunication(online,plotting);
    % Race setup
    dist_goal = 402;
    telemetry.Distance = 0;
    telemetrylog = [];
    prevshifttime = 1;
	initialRun = 1;
    while(telemetry.Distance <= dist_goal || online)
        if(initialRun) % should ask for gear before a timestep is taken
			[gear_demand, throttle] = student_AutomaticGearbox(car.dyn.gear, car.dyn.rpm,...
                car.dyn.acc, car.dyn.speed, car.dyn.throttle, car.dyn.dist, car.dyn.time);
			initialRun = 0;
        else
             % Get Telemetry
            [telemetry, car] = caster_GetTelemetry(car, udpInterfaceTelemetry,plotting);

            telemetrylog = [telemetrylog, telemetry]; %#ok<AGROW>

            [gear_demand, throttle] = student_AutomaticGearbox(telemetry.Gear, telemetry.RPM, ...
                    telemetry.LongAcc, telemetry.Velocity, telemetry.Throttle, ...
                    telemetry.Distance, telemetry.TimeLap);           
        end        				

		if(plotting)
            if(gear_demand == 0 || gear_demand == -1)
               warning('You have tried to put in reverse or neutral. While this works physically, the simulation will probably get stuck in an infinite loop since the car won''t reach the finish line')
            end
            if(gear_demand < -1)
                warning(['You have tried to put in the gear ', num2str(gear_demand),'. A car normally doesn''t have multiple reverese gears. This is a car falling into that category. Your gear choice are changed to the normal reverse gear. Since the finish line is forward, not rearward, the simulation will probably be stuck in an infinte loop from now on'])
            end
            if(gear_demand > car.gearbox.gears)
                warning(['You have tried to put in the gear ', num2str(gear_demand),'. This car only has ',...
                    num2str(car.gearbox.gears), ' gears. Did you think you could cheat? Picking gear ', num2str(car.gearbox.gears), ' instead'])
            end
            if(throttle > 1)
                warning('You have tried to give a throttle higher than 1, which would be equal to pressing the pedal through the floor. Since I don''t want holes in the floor, i will change the throttle to 1')
            end
            if(throttle < 0)
                warning('You have tried to give a throttle lower than 0, which would be equal to ripping the pedal loose. Since I want to keep the pedal I will change the throttle to 0')
            end
            if(throttle == 0)
                warning('You aren''t pressing the throttle. This will stop the car. A car that is stationary won''t win dragraces. You are probably watching the simulation stuck in an infinite loop right now')
            end
            if(round(gear_demand) ~= gear_demand)
                warning(['You have requested gear ', num2str(gear_demand), '. This would be equal to almost putting the gear in. If you have driven a car, you know which awful noise that will result in. I don''t like aweful noices so I''ve fixed the gear request for you. You are now requesting gear ', num2str(round(gear_demand))])
            end
        end
        if(gear_demand < -1)
            gear_demand = 0;
        end
        gear_demand = round(gear_demand);
        car.dyn.throttle = throttle;
        % Shift gear
        [car, prevshifttime] = caster_GearShift(gear_demand, prevshifttime, ...
                                                car, udpInterfaceConsole,plotting);
        disp("");
    end
    
    if(plotting)
        % Plotting
        figure(1)
        hold all
        subplot(2,2,1)
        plot([telemetrylog.Distance],[telemetrylog.Velocity]*3.6,'r.')
        xlabel 'Distance [m]'
        ylabel 'Velocity [km/h]'
        subplot(2,2,2)
        plot([telemetrylog.Distance],[telemetrylog.RPM],'b.')
        xlabel 'Distance [m]'
        ylabel 'Engine RPM'
        subplot(2,2,3)
        plot([telemetrylog.Distance],[telemetrylog.Gear],'k.')
        xlabel 'Distance [m]'
        ylabel 'Current Gear'
        subplot(2,2,4)
        plot([telemetrylog.Distance],[telemetrylog.LongAcc],'g.')
        xlabel 'Distance [m]'
        ylabel 'Longitudinal Acceleration [m/s^2]'
    end
    finishtime = interp1([telemetrylog.Distance],[telemetrylog.TimeLap],dist_goal);
    disp([newline 'Finish time: ' num2str(finishtime),' seconds'])
end

function [car, prevshifttime] = caster_GearShift(gear_demand,prevshifttime,car,udpInterfaceConsole, plotting)

    versionnr=1.3;
    if(prevshifttime == 0 && plotting)
        disp(['GearShift Version ' num2str(versionnr,'%1.1f')])
        prevshifttime=car.dyn.time;
        
    end
    mintimeShift=car.gearbox.shiftingtime;
    
    % Limit to allowed gears
    gear_demand = min(gear_demand,car.gearbox.gears);
    gear_demand = max(-1, gear_demand);
    
    % Check so that the desired shift time has passed
    if car.dyn.time-prevshifttime >= mintimeShift
        % Update gear in the model
        if gear_demand ~= car.dyn.gear
            % Make an arbitrary change to neutral to simulate clutch
            if(car.dyn.gear ~= -1)
                % Check if running online
                if(isempty(udpInterfaceConsole))
                    car.dyn.gear = -1;
                else
                    fopen(udpInterfaceConsole);
                    if(car.dyn.gear == 0)
                        fwrite(udpInterfaceConsole,'shiftup')
                    else
                        for i = 1:car.dyn.gear
                            fwrite(udpInterfaceConsole,'shiftdown')
                        end
                    end
                    fclose(udpInterfaceConsole);
                end
                prevshifttime = car.dyn.time;
                car.dyn.gotogear = gear_demand;
            else
                if(isempty(udpInterfaceConsole))
                    car.dyn.gear = car.dyn.gotogear;
                else
                    fopen(udpInterfaceConsole);
                    if(car.dyn.gotogear == 0)
                        fwrite(udpInterfaceConsole,'shiftdown')
                    else
                        for i = 1:car.dyn.gotogear
                            fwrite(udpInterfaceConsole,'shiftup')
                        end
                    end
                    fclose(udpInterfaceConsole);
                end
                prevshifttime = car.dyn.time;
            end
        end
    end
end
       
function [telemetry, car] = caster_GetTelemetry(car, udpInterfaceTelemetry,plotting)
    
    versionnr = 1.4;

    % Sample time
    if(car.dyn.time == 0 && plotting)
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
    %Simulates and updates dynamic parameters of car.

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

    %   � CASTER / Eric Gunnarsson
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

function [car, udpInterfaceTelemetry, udpInterfaceConsole] = caster_InitCommunication(online,plotting)

    versionnr = 1.5;
    
    %% Design parameters
    if(plotting)
        disp(['InitCommunication Version ' num2str(versionnr,'%1.1f')])
    end
    
    if(online)
        disp('Running online, checking if config.ini exists')
        try
            config = caster_iniReader('config.ini');
            disp('config.ini loaded')
        catch ME
            if(strcmp(ME.identifier,'MATLAB:FileIO:InvalidFid'))
                msg = 'config.ini is missing, new one is created. Please fill out and restart script';
                f = fopen('config.ini','w');
                template = 'master\n{\n\tip = 129 16 66 32\n\ttelemetry_port = 7000\n\tconsole_port = 26000\n}';
                template = [template, '\nlocal\n{\n\ttelemetry_port = 7000\n\tconsole_port = 26000\n}'];
                fprintf(f,template);
                fclose(f);
                error(msg);
            end
            rethrow(ME)
        end
        
        % Network clear
        instrreset;

        %% Network setup
        timeoutLimit = 0.01;
        timerPeriod = 0.01;
        master_ip = [num2str(config.master.ip(1)), '.',...
            num2str(config.master.ip(2)), '.',...
            num2str(config.master.ip(3)), '.',...
            num2str(config.master.ip(4))];
        
        disp(['Connecting to master at ip: ', master_ip])
        % Define interface
        udpInterfaceTelemetry = udp(master_ip,config.master.telemetry_port,...
                   'LocalPort',config.local.telemetry_port,...
                   'Timeout',timeoutLimit,...
                   'byteorder','littleendian');

        %% Define interface for console
        udpInterfaceConsole = udp(master_ip,config.master.console_port,...
                   'Timeout',timeoutLimit,...
                   'TimerPeriod',timerPeriod,...
                   'byteorder','littleendian');

    else
        udpInterfaceConsole = [];
        udpInterfaceTelemetry = [];    
    end
    
    % Tires
    car.pacejka.b0 = 1.6;
    car.pacejka.b1 = -58;
    car.pacejka.b2 = 1750;
    car.pacejka.b3 = 6.8;
    car.pacejka.b4 = 200;
    car.pacejka.b5 = 0;
    car.pacejka.b6 = 0.0034;
    car.pacejka.b7 = -0.008;
    car.pacejka.b8 = -0.76;
    car.pacejka.b9 = 0;
    car.pacejka.b10 = 0;
    car.pacejka.b11 = 0;
    car.pacejka.b12 = 0;
    
    % Engine
    car.engine.rpm =    [   0       511.111 1211.11 2044.45 ...
                            4166.66 5200    5533.34 5922.22 ...
                            6322.22 6500    6955.56 7488.89 ...
                            7500    7922.22 8000    8300    ...
                            8500    10000   10500];
	
    car.engine.torque = [  0.1894    0.2030    0.2421    0.2740 ...
                            0.3784    0.8142    0.8702    0.9501 ...
                            0.9994    0.9994 0.9450    0.9025 ...
                            0.9242    0.8668    0.9027    0.7887  ...
                           0.8597         0         0]*864;
	
    car.engine.max_rpm = 8500;
    car.engine.idle_rpm = 1850;
    
    % Gearbox
    car.gearbox.gears = 4;
    car.gearbox.ratio = [3.8 2.3 1.4 0.95];
    car.gearbox.final = 4.03;
    car.gearbox.shiftingtime = 0.1;
    
    % Wheels
    car.wheel.inertia = 2.04+2.05/2+0.1/2+0.05; % Wheel, engine,gearbox, driveshaft inertia
    car.wheel.radius = 0.32;
    car.wheel.rollcoeff = 0.015;
    
    % Chassis
    car.chassis.mass = 1030+46+36+30;
    car.chassis.cog_height = 0.4;
    car.chassis.wheel_base = 2.273;
    
    % Aero
    car.aero.frontal_area = 1.9;
    car.aero.down_coefficient = 0.1;
    car.aero.drag_coefficient = 0.3;
    car.aero.air_density = 1.225;

    %% Output variables
    dyn.rpm = car.engine.idle_rpm;
    dyn.speed = 0;
    dyn.acc = 0;
    dyn.dist = 0;
    dyn.time = 0;
    dyn.w_wheel = 0;
    dyn.prevshifttime = -1;
    dyn.gotogear = 1;
    dyn.gear = 1;
    dyn.throttle = 0;
    car.dyn = dyn;
end