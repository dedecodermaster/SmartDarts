%% Définition des paramètres
g = 9.81; % m/s^2
y0 = 1.6; % m
z0 = 2.3; % règle fléchette
v0 = 16; % m/s
theta = 3; % degrès
deltat = 1/120; % Fréquence des caméras
t = 0:deltat:0.15;


F = 0;
m = 23e-3;

%% Modélisation
y = v0*t*sin(theta*2*pi()/360)-(1/2)*g*(t.^2)+y0;
z = z0 - v0*t*cos(theta*2*pi()/360);

%% Récupération de la liste de points
T = [z;y];
s = size(T);
%figure
plot(z,y,'or','MarkerSize',12);
writematrix(T,'modele2d.txt','Delimiter',';')  
hold on

%% Reconstruction
%[zp,yp] = Euler2emeordre(z,y,deltat,F,m,g,t);
lip = [];
mlip = 3;
for k = round(length(t)/10):length(t) % boucle qui reproduit les différents temps de mesure
    "Nouvelle boucle";
    Tk = [z(1:k);y(1:k)]; % Liste des mesures
    zk = [Tk(1,1); Tk(2,1); ((Tk(1,2)-Tk(1,1))/deltat); ((Tk(2,2)-Tk(2,1))/deltat)+0.04]; % Prédiction voulue au départ = conditions initiales
    Gzk = cov(zk)*10;
    Gzk_pred = Gzk/10;
    uk = [0; 0; 0; 0]; % Entrée du système (pour l'instant aucune force extérieures
    Ak = [1 0 deltat 0; 0 1 0 deltat*(1-(1/2)*9.81);0 0 1 0;0 0 0 1]; % Matrice de passage entre l'état actuel et l'état suivant
    Galphak = [0.0015 0 0 0; 0 0.0015 0 0; 0 0 0 0; 0 0 0 0]; % Précision de la caméra
    yk = Tk + 0.0005*randn(size(Tk));
    Ck = [1 0 0 0; 0 1 0 0]; % On ne mesure que les positions
    Gbetak = [0.005 0; 0 0.005]; % on autorise une erreur de 5 millimètres selon les deux axes, sinon on a peu de prédiction
    [zk,Gzk,zkup,Gzkup] = testKalman(zk,Gzk,uk,yk,Galphak,Gbetak,Ak,Ck);
    "sortie";
    % ykpr = zk;
    % Gpr = Gzk;
    for j = k:length(t)-1 % Création d'une trajectoire simulée
        yc = zkup(4,1)*t(j+1:length(t))-(1/2)*g*(t(j+1:length(t)).^2) + Tk(2,1)
        zc = Tk(1,1) + zkup(3,1)*t(j+1:length(t))
        plot(zc,yc,':');
        new_y = [zc; yc];
        yk = [yk new_y]; % Définition de la trajectoire simulée comme la mesure
        zk = [zkup zk(:,j)];
        [zk,Gzk_pred,zkup,Gzkup] = testKalman(zk,Gzk_pred,uk,yk(:,1:length(zk(1,:))),Galphak,Gbetak,Ak,Ck)
        % ykpr = zk;
        % Gpr = Gzk;
        % zkup(:,j) = zk(:,k-1);
        % 
        % zkup(1,j) = zc;
        % zkup(2,j) = yc;  
    end    
    plot(zkup(1,:),zkup(2,:),'--');  

%% Récupération des points d'impact
    for i = 2:length(zkup)
        if ((zkup(1,i-1)>0) && (zkup(1,i)<0))
            a = (zkup(2,i)-zkup(2,i-1))/(zkup(1,i)-zkup(1,i-1));
            b = zkup(2,i-1) - a*zkup(1,i-1);
            if k == round(length(t)/10)
                lip = [lip b]
            elseif b>=0.99*mlip && b<=1.01*mlip && i>2 % j'enlève les valeurs abérantes
                lip = [lip b]
            end
            mlip = mean(lip)
        end
    end
end
hold off
%axis([0 z0 0 2]);