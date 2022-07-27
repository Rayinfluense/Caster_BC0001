function gear_demand = caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)

    %   © CASTER / Eric Matsson
    %   Created: 2016-06-05
    %   Last edit: 2018-08-28 by Richard L
    %   gear_demand = caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)
    %
    %   Calculate the "optimal" gear at any given time given the telemetry data
    %   from the vehicle.
    %   Utilizes engine torque-map and gear ratios to calculate shifting points
    %   and compares this to the current state of the vehicle in order to
    %   select the "best" gear.
    % 
    %   Define the global variables to get access to the car data.

    versionnr = 1.3;
    
    global firstRun vehicleData
    
    if exist('vehicleData.mat','file') == 2
        if(isempty(firstRun))
            firstRun = 0;
            vehicleData = load('vehicleData.mat');
        end
    else
        if(isempty(firstRun))
            firstRun = 0;
            disp('!! No vehicleData.mat found. Please add it to use caster_AutomaticGearbox !!');
        end
        gear_demand = 1;
        return
    end

    numGears = size(vehicleData.gearRatios,2);

    % Used only first time the function is called in order to save calculation
    % time.
    if ~any(~cellfun(@isempty,strfind(fieldnames(vehicleData),'shiftupvelocities')))
        disp(['caster_AutomaticGearbox Version ' num2str(versionnr,'%1.1f')])

        rpmResolution = 1;

        % RPM vector
        rpm = vehicleData.rpmVector;

        % Torque vector
        torqueFactor = vehicleData.torqueVector;
        gearRatios=[0, vehicleData.gearRatios];

        % Create empty variables to be filled later       
        speedRatios=zeros(numGears-1);
        shiftupRPMs=zeros(numGears-1,1);
        shiftdownRPMs=zeros(numGears-1,1);

        % Calculate a power relationship between the gears
        powerFactor=rpm/30*pi().*torqueFactor/1000;

        % Calculate the speed relationship between the gears
        for i=2:numGears
            speedRatios(i,1)=gearRatios(i+1)/gearRatios(i);
        end

        % Create a vector of RPMs that will be compared between the gears
        rpms=(max(rpm)/2:rpmResolution:max(rpm))';

        % Calculate the rpm for when two different gears produce the same power
        for i=2:length(speedRatios)
            lowGear=interp1(rpm,powerFactor,rpms,'linear');
            highGear=interp1(rpm,powerFactor,rpms*speedRatios(i),'linear');
            shiftupRPMs(i-1)=rpms(find(highGear>lowGear,1));
            shiftdownRPMs(i)=rpms(find(highGear>lowGear,1))*speedRatios(i);
        end

        % Calculate which velocities correspons to the calculated shifting rpms
        vehicleData.shiftupvelocities = shiftupRPMs./gearRatios(2:end-1)'/vehicleData.finalDriveRatio/60*vehicleData.wheelRadius*2*pi();
        vehicleData.shiftdownvelocities = shiftdownRPMs./gearRatios(2:end)'/vehicleData.finalDriveRatio/60*vehicleData.wheelRadius*2*pi()*0.95;
    end

    % Get current gear
    gear_demand = Gear;

    % Check so that you are not in neutral or reverse
    if gear_demand ~= -1
        % Check if the velocity is higher/lower than the shifting velocity for
        % the current gear and if that is the case shift the gear one step in
        % the desired direction.
        if Gear < numGears && Velocity > vehicleData.shiftupvelocities(Gear)
            gear_demand = Gear+1;
        elseif Gear > 1.1 && Velocity < vehicleData.shiftdownvelocities(Gear)
            gear_demand = Gear-1;
        end

    % Just a fail safe for the simulator to not get stuck in neutral/reverse
    else
        gear_demand=4;
    end

end