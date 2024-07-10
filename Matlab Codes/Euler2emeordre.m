%% Méthode
function [zp,yp] = Euler2emeordre(z,y,deltat,F,m,g,t)
    zp = zeros(1,length(t));
    yp = zeros(1,length(t));
    vz = zeros(1,length(t));
    vy = zeros(1,length(t));
    zp(1) = z(1);
    yp(1) = y(1);
    vz(1) = (z(2)-z(1))/deltat;
    vy(1) = (y(2)-y(1))/deltat;
    for n = 1:length(t)-1
        zp(n+1) = zp(n) + vz(n)*deltat;
        yp(n+1) = yp(n) + vy(n)*deltat;
        vz(n+1) = (zp(n+1)-zp(n))/deltat;
        vy(n+1) = (yp(n+1)-yp(n))/deltat;
    end
end


