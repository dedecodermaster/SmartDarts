function [x1,G1] = predKalman(x0,G0,u,Galpha,A)
    x1 = A*x0 + u;
    G1 = A*G0*transpose(A)+Galpha;
end