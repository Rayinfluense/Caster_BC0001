function [car, prevshifttime] = caster_GearShift(gear_demand,prevshifttime,car,udpInterfaceConsole)

    versionnr=1.3;
    if(prevshifttime == 0);
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
                    fclose(udpInterfaceConsole)
                end
                prevshifttime = car.dyn.time;
            end
        end
    end
end
       