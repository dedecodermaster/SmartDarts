function [x1,G1,xup,Gup] = testKalman(x0,G0,u,y,Galpha,Gbeta,A,C) %#codegen
% x0 l'estimation précédente
% G0 la covaraince précédente
% u les entrées (pour nous aucune)
% y les mesures
% A = F 
% C = H
 % arguments
 %     x0 (6,:) double
 %     G0 (6,6) double
 %     u (6,1) double
 %     y (3,:) double
 %     Galpha (6,6) double
 %     Gbeta (3,3) double
 %     A (6,6) double
 %     C (3,6) double
 %     i double
 % end
n = size(y,2);
if (y == zeros(3,n))
    y = eye(3,n);
    Gbeta = eye(3);
    C = eye(3,6);
end
% if (i==1)
%     S = C.*G0*transpose(C)+Gbeta;
%     K = transpose(C)*inv(S).*G0;
%     Gup = G0-K*C.*G0;
% else
    S = C*G0*transpose(C)+Gbeta;
    K = G0*transpose(C)*inv(S);
    Gup = G0-K*C*G0;
% end
ytilde = y-C*x0;
xup = x0 + K*ytilde;
x1 = A*xup + u;
G1 = A*Gup*transpose(A)+Galpha;
end

