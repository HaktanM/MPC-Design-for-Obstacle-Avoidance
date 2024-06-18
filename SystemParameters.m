function [CarMass, CarInertia, surface_fc, air_fc, distance_a, distance_b] = SystemParameters()

CarMass = 2050;   % kg

CarInertia = 3344;   % kg m^2

% fc stands for friction coefficient
surface_fc = 0.3;  

p = 1.225;   % kg / m^3    ---    air density
Cd = 0.31;   % aerodynamic drag coefficient
Sd = 4;      % m^2   ---   Vehicle frontal crossection

% fc stands for friction coefficient
air_fc = 0.5 * p * Cd * Sd; % kg/m 

CarLength = 2;   %meters
CoG_Location_Parameter = 0.55;

distance_a = CarLength * CoG_Location_Parameter;
distance_b = CarLength * (1 - CoG_Location_Parameter);

end