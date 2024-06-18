function u = MPC_Controller_with_Ricatti(A,B,Hl,x0,Q,R,x,xd,StepLength)

B_length = length(B);
z = zeros(B_length,1);
G = [];
for i = (1:Hl+1) 
    row = [];
    for j = 1:Hl
        if j-i >= 0
            row = [row z];
            else if i>(j+1)
                row = [row ((A^(i-1-j))*B)];
                else
                    row = [row B];
            end
        end
    end
    G = [G; row];
        
end

H = [];
for i = 1 : (Hl+1)
    H = [H; A^(i-1)];
end



% Ricatti - P_inf
[P_inf,L,K] = dare(A,B,Q,R);

Qf = P_inf;

QB = [];
RB = [];
for i = 1:Hl
    QB = blkdiag(QB, Q);
    RB = blkdiag(RB, R);
end

QB = blkdiag(QB,Qf);



% Optimization Matrices
M = transpose(G) * QB * G + RB;
alphaT = transpose(x0) * transpose(H) * QB * G;


% Obstacle Definition and Avoidance
UpperLimit = CalculateUpperLimits(x,xd,Hl,StepLength);
LowerLimit = CalculateLowerLimits(x,xd,Hl,StepLength);

Hx0 = [
    -H*x0
    H*x0
];

LimitMatrix = [
    UpperLimit
    LowerLimit
];

LimitMatrix = LimitMatrix + Hx0;
StateEquations = [G; -G];

warning off all
options = optimoptions('quadprog','Display','off');
u = quadprog(M,alphaT',StateEquations,LimitMatrix,[],[],[],[],[],options);  %  Saturation is respected
warning on

end


