clearvars -except A B  
clc

A = eye(5);
T = 1;
Ad = expm(A*T);

fun = @(x) expm(A*x)*B;
q = integral(fun,0,T,'ArrayValued', 1);

