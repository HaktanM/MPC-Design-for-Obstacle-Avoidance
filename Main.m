close all
clear
clc

% Simulation Parameters
SimulationLength = 40;   % seconds
StepLength = 0.1;        % seconds
FullHorizon = floor(SimulationLength/StepLength);

% Horizon Length
Hl = 100;

% Horizontal Steady State Speed
OperationSpeed = 20;

% Estimated road radius
r_estimated = 500;

% Initial H-Position of the Car
horizontal_position = 0;

% Initial Condition
x0 = [
  0
  0
  0
  0
  0
];

Q = [
    0.04 0 0 0 0
    0 0.62 0 0 0
    0 0 205.18 0 0
    0 0 0 0.29 0
    0 0 0 0 10.13
];

Qf = 100 * Q;
R = 2.53;

% MPC Design
X = zeros(length(x0),FullHorizon);
X(:,1) = x0;

U = zeros(1,FullHorizon-1);

x = zeros(1,FullHorizon);
y = zeros(1,FullHorizon);

x(1) = horizontal_position;

road_radius = r_estimated;
xd = OperationSpeed;

for i = 1:(FullHorizon-1)
    i
    xd = xd +  OperationSpeed*0.0*(0.5-rand);
    road_radius = road_radius + r_estimated*0.02*(0.5-rand);
    [A, B, state_offset] = state_space_LTV(xd,road_radius);
    [Ad, Bd] = ObtainDiscreteModel(A,B,StepLength);
    %Ad = eye(length(A)) + A*StepLength;
    %Bd = B * StepLength;

    xi = X(:,i);
    u = MPC_Controller_with_Ricatti(Ad,Bd,Hl,xi,Q,R,x(i),xd,StepLength);
    %u = MPC_Controller(Ad,Bd,Hl,xi,Q,Qf,R,x(i),xd,StepLength);
    
    U(i) = u(1);
    X(:,i+1) = Ad*X(:,i) + Bd*U(i);
    x(i+1) = x(i) + xd * StepLength;
    y(i) = X(4,i);

end
x(length(x)) = x(length(x)-1);
state_offset = zeros(1,length(state_offset));
CreatePlots(FullHorizon,StepLength,X,U,state_offset,x,y)



