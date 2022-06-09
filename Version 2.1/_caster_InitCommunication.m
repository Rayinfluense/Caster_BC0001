function [car, udpInterfaceTelemetry, udpInterfaceConsole] = caster_InitCommunication(online)

    versionnr = 1.4;
    
    %% Design parameters
    disp(['InitCommunication Version ' num2str(versionnr,'%1.1f')])
    
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
    car.pacejka.b2 = 2050;
    car.pacejka.b3 = 6.8;
    car.pacejka.b4 = 440;
    car.pacejka.b5 = 0;
    car.pacejka.b6 = 0.0034;
    car.pacejka.b7 = -0.008;
    car.pacejka.b8 = 0.66;
    car.pacejka.b9 = 0;
    car.pacejka.b10 = 0;
    car.pacejka.b11 = 0;
    car.pacejka.b12 = 0;
    
    % Engine
    car.engine.rpm = [0, 500, 1000, 1500, 2050, 2550, 2930, 3350, 3760, ...
                     4370, 5040, 5610, 6110, 6570, 7160, 7570, 7980, ...
                     8400, 8500, 9000, 9500];
	
    car.engine.torque = [0.3199, 0.3199, 0.3839, 0.4095, 0.4415, 0.5566, ...
                        0.7255, 0.8868, 0.9808, 1.0000, 0.9827, 0.9501, ...
                        0.8983, 0.8484, 0.7812, 0.7159, 0.6276, 0.3916, ...
                        0.3199, 0, 0]*900;
	
    car.engine.max_rpm = 8500;
    car.engine.idle_rpm = 1850;
    
    % Gearbox
    car.gearbox.gears = 4;
    car.gearbox.ratio = [3.21, 1.8, 1.25, 0.95];
    car.gearbox.final = 4.19;
    car.gearbox.shiftingtime = 0.1;
    
    % Wheels
    car.wheel.inertia = 2.04+2.05/2+0.1/2+0.05; % Wheel, engine,gearbox, driveshaft inertia
    car.wheel.radius = 0.32;
    car.wheel.rollcoeff = 0.015;
    
    % Chassis
    car.chassis.mass = 1417+46+36;
    car.chassis.cog_height = 0.4;
    car.chassis.wheel_base = 2.7;
    
    % Aero
    car.aero.frontal_area = 1.5;
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
    dyn.throttle = 1;
    car.dyn = dyn;
end