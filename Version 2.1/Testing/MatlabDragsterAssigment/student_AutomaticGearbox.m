function [gear_demand, throttle] = student_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)    
%% En funktion som bestämmer vilken växel som skall användas baserat på 7 variabler. 
% Av alexander Hägglund 2018-09


% Convert m/s to km/h
    Speed = Velocity * 3.6;
    
    clc
    
  
    load vehicleData.mat
    %% Beräkningar om bilen för antispinnsystem
 if Gear > 0
     
     
    RPM_wheel=3.6*RPM./(gearRatios(Gear)*finalDriveRatio);
  
    wheelspeed=(2*pi/60)*wheelRadius*RPM_wheel;
 end
    %% Check the speed and determine gear
    % konstigt att det går fortare om man inte använder växel 2!!!,
    % Varför??
    if Speed > 177
        gear_demand = 4;
  
        elseif Speed > 120
        gear_demand = 3;
    
        elseif Speed > 73
        gear_demand = 3;
    
        else  
        gear_demand = 1;
    end
   %% Förhindrar att bilen växlar ner precis vid nedväxling
    if gear_demand < Gear
       gear_demand = Gear;
    end
    
    
   %% Antispinn system

   
   
   if Gear > 0
  
       if Speed*1.12 < wheelspeed && Throttle ~=0
    Throttle=Throttle-0.122;
       end
       
       if Speed*1.12 > wheelspeed && Throttle ~=0
    Throttle= Throttle+0.126;
       end
       
   end
   %% Förhindrar negativ Gas
   throttle = Throttle;
       
       if throttle <0.25
           throttle=0.25;
       end
       %% Den komentarmarkerade koden nedan kan förhindra felkoden som sker vid körning av programmet då gasen blir större än 1!
% 
%        if throttle > 1
%            throttle=1;
%        end

   
   %% For comparasion, uncomment the line below to see how caster gearbox performs
    % gear_demand = caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap);end 