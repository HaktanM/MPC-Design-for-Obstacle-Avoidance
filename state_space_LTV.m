function [A, B, state_offset] = state_space_LTV(xd,r)

% Get the system parameters
[m, I, mu, kd, a, b] = SystemParameters;
% m is the mass of the car
% I is the angular inertia of the car around CoG
% mu is the friction coefficient between car and ground
% kd is the friction coefficient between car and air
% r is the radius of the road
% a is the distance of CoG to front of the car
% b is the distance of CoG to back of the car

% Gravitationla Acceleration
g = 9.8; % m/sec^2


[C_fL, C_fU, C_rL, C_rU] = C_Parameters();

A = [
    (C_fL+C_rL)/(m*xd) (a*C_fL-b*C_rL)/(m*xd)-xd 0 0 -C_fL/m
    (a*C_fL-b*C_rU)/(I*xd) ((a^2)*C_fL+(b^2)*C_rU)/(I*xd) 0 0 -a*C_fL/I
    0 1 0 0 0
    1 0 xd 0 0
    0 0 0 0 0
    ];

B = [
    0
    0
    0
    0
    1
];

OffSett = [
    0
    0
    -xd/r
    0
    0
];



% Eliminate the offset by changing the variables
aM = [
    A(1,1) A(1,5)
    A(2,1) A(2,5)
];


if ~det(aM) == 0
    c2 = -OffSett(3);
    offset_equation_rhs = [
    -A(1,2)*c2  
    -A(2,2)*c2 
    ]; 
    small_state_offset = inv(aM) * offset_equation_rhs;
    state_offset = [
    small_state_offset(1)
    c2
    -small_state_offset(1)/xd
    0
    small_state_offset(2)
    ];
else
    fprintf('The algorithm is not able to get rid of the offset')
end

check = prod ( ( (A*state_offset+OffSett) < 1e-12 ) , 'all');

if ~(check == 1)
    fprintf('The offset elimination fails for some reason only god knows why. \n')
end

end