function [gear_demand, throttle] = student_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap)    
%% En funktion som best�mmer vilken v�xel som skall anv�ndas baserat p� 7 variabler. 
% Av alexander H�gglund 2018-09


% Convert m/s to km/h
    Speed = Velocity * 3.6;
    
    clc
    
  
    load vehicleData.mat
    %% Ber�kningar om bilen f�r antispinnsystem
 if Gear > 0
     
     
    RPM_wheel=3.6*RPM./(gearRatios(Gear)*finalDriveRatio);
  
    wheelspeed=(2*pi/60)*wheelRadius*RPM_wheel;
 end
    %% Check the speed and determine gear
    % konstigt att det g�r fortare om man inte anv�nder v�xel 2!!!,
    % Varf�r??
    if Speed > 177
        gear_demand = 4;
  
        elseif Speed > 120
        gear_demand = 3;
    
        elseif Speed > 73
        gear_demand = 3;
    
        else  
        gear_demand = 1;
    end
   %% F�rhindrar att bilen v�xlar ner precis vid nedv�xling
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
   %% F�rhindrar negativ Gas
   throttle = Throttle;
       
       if throttle <0.25
           throttle=0.25;
       end
       %% Den komentarmarkerade koden nedan kan f�rhindra felkoden som sker vid k�rning av programmet d� gasen blir st�rre �n 1!
% 
%        if throttle > 1
%            throttle=1;
%        end

   
   %% For comparasion, uncomment the line below to see how caster gearbox performs
    % gear_demand = caster_AutomaticGearbox(Gear, RPM, LongAcc, Velocity, Throttle, Distance, TimeLap);end 