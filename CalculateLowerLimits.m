function  State_Limits = CalculateLowerLimits(x,xd,Hl,StepLength)
% x is the forizontol location of the car
% xd is the horizontol velocity of the car
% Hl is the horizontol length
% SteoLength is the time interval between two evaluation


[ObstacleLocation, ObstacleLength, ObstacleWidth] = defineObstacle;


% Number of States Before Arrival to the Obstacle
Arrival = (ObstacleLocation-x+ObstacleLength+2*xd) / (xd * StepLength);



% Number of States Before Leaving the Obstacle
Leaving = (ObstacleLocation+ObstacleLength-x+5*xd) / (xd * StepLength);

State_Limits = [];

InfM = [
    Inf
    Inf
    Inf
    Inf
    pi/10
];



LimitM = [
    Inf
    Inf
    Inf
    Inf
    pi/10
];




for i = 1 : (Hl+1)
    if ((i>=Arrival)&&(i<=Leaving))
        State_Limits = [State_Limits; LimitM];
    else
        State_Limits = [State_Limits; InfM];
    end
end


5+5;
end

