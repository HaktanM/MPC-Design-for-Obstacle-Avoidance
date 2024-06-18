function [Ad, Bd] = ObtainDiscreteModel(A,B,StepLength)
    Ad = expm(A*StepLength);
    fun = @(x) expm(A*x)*B;
    Bd = integral(fun,0,StepLength,'ArrayValued', 1);

end