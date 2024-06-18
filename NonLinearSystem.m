close all
clear
clc

% Simulation Parameters
SimulationLength = 10;   % seconds
StepLength = 0.1;        % seconds
FullHorizon = floor(SimulationLength/StepLength);

% Horizon Length
Hl = 100;

% Horizontal Steady State Speed
OperationSpeed = 10;

% Initial H-Position of the Car
horizontal_position = 0;

% Initial Condition
x0 = [
  0
  0
  0
  1
  0
];

Q = [
    10 0 0 0 0
    0 0 0 0 0
    0 0 0 0 0
    0 0 0 100 0
    0 0 0 0 0
];

Qf = 1 * Q;
R = 0.0001;

% MPC Design
X = zeros(length(x0),FullHorizon);
X(:,1) = x0;

U = zeros(1,FullHorizon-1);

x = zeros(1,FullHorizon);
y = zeros(1,FullHorizon);

x(1) = horizontal_position;


for i = 1:(FullHorizon-1)
    xd = OperationSpeed; %* (1+0.02*rand);    %+4*(0.5-rand);
    ConstantTerm = [0; 0; 0; -xd/500; 0];

    [A, B, state_offset] = state_space_LTV(xd,500);
    [Ad, Bd] = ObtainDiscreteModel(A,B,StepLength);
    %Ad = eye(length(A)) + A*StepLength;
    %Bd = B * StepLength;
    
    xi = X(:,i);
    u = MPC_Controller(Ad,Bd,Hl,xi,Q,Qf,R,x(i),xd,StepLength);
    U(i) = u(1);
    X(:,i+1) = Ad*X(:,i) + Bd*U(i) + ConstantTerm;
    x(i+1) = x(i) + xd * StepLength;
    y(i) = X(4,i);

end

x(length(x)) = x(length(x)-1);

CreatePlots(FullHorizon,StepLength,X,U,state_offset,x,y)



