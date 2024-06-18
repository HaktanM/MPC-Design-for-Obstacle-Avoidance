function  State_Limits = CalculateUpperLimits(x,xd,Hl,StepLength)
% x is the forizontol location of the car
% xd is the horizontol velocity of the car
% Hl is the horizontol length
% SteoLength is the time interval between two evaluation


[ObstacleLocation, ObstacleLength, ObstacleWidth] = defineObstacle;



% Number of States Before Arrival to the Obstacle
Arrival = (ObstacleLocation-x) / (xd * StepLength);



% Number of States Before Leaving the Obstacle
Leaving = (ObstacleLocation+ObstacleLength-x) / (xd * StepLength);

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
    -0.55*ObstacleWidth
    pi/10
];




for i = 1 : (Hl+1)
    if ((i-Arrival>=0)&&(i-Leaving<=0))
        State_Limits = [State_Limits; LimitM];
    else
        State_Limits = [State_Limits; InfM];
    end
end

end

