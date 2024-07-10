%% Récupération des données
mesures = readtimetable('essaigrandefléchette.csv')

%% Récupération de la liste de points
T = [z;x;y]; % à changer avec les bonnes lignes et colonnes
s = size(T);
%figure
plot3(z,x,y,'or','MarkerSize',12);
hold on

% Reconstruction
%[zp,yp] = Euler2emeordre(z,y,deltat,F,m,g,t);
lip = [];
mlip = [3;0.5];
for k = round(length(t)/10):length(t) % boucle qui reproduit les différents temps de mesure
    "Nouvelle boucle";
    Tk = [z(1:k);x(1:k);y(1:k)]; % Liste des mesures
    zk = [Tk(1,1); Tk(2,1); Tk(3,1); ((Tk(1,2)-Tk(1,1))/deltat); ((Tk(2,2)-Tk(2,1))/deltat); ((Tk(3,2)-Tk(3,1))/deltat)+0.04]; % Prédiction voulue au départ = conditions initiales
    Gzk = cov(zk)*10;
    uk = [0; 0; 0; 0; 0; 0]; % Entrée du système (pour l'instant aucune force extérieures
    Ak = [1 0 0 deltat 0 0; 0 1 0 0 deltat 0; 0 0 1 0 0 deltat*(1-(1/2)*9.81);0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]; % Matrice de passage entre l'état actuel et l'état suivant
    Galphak = [0.0015 0 0 0 0 0; 0 0.0015 0 0 0 0; 0 0 0.0015 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]; % Précision de la caméra
    yk = Tk + 0.0005*randn(size(Tk));
    Ck = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]; % On ne mesure que les positions
    Gbetak = [0 0 0; 0 0 0; 0 0 0]; % on autorise une erreur de 5 millimètres selon les deux axes, sinon on a peu de prédiction
    [zk,Gzk,zkup,Gzkup] = testKalman(zk,Gzk,uk,yk,Galphak,Gbetak,Ak,Ck);
    "sortie";
    % ykpr = zk;
    % Gpr = Gzk;
    for j = k:length(t)-1 % Création d'une trajectoire simulée
        yc = zkup(6,1)*t(j+1:length(t))-(1/2)*g*(t(j+1:length(t)).^2) + yk(3,1);
        zc = yk(1,1) + zkup(4,1)*t(j+1:length(t))
        xc = yk(2,1) + zkup(5,1)*t(j+1:length(t));
        %plot3(zc,xc,yc,':');
        new_y = [zc; xc; yc];
        yk = [yk new_y]; % Définition de la trajectoire simulée comme la mesure
        zk = [zkup zk(:,j)];
        [zk,Gzk,zkup,Gzkup] = testKalman(zk,Gzk,uk,yk(:,1:length(zk(1,:))),Galphak,Gbetak,Ak,Ck)
    end    
    plot3(zkup(1,:),zkup(2,:),zkup(3,:),'--');

%% Récupération des points d'impact
    for i = 2:length(zkup)
        if ((zkup(1,i-1)>0) && (zkup(1,i)<0))
            ay = (zkup(3,i)-zkup(3,i-1))/(zkup(1,i)-zkup(1,i-1));
            by = zkup(3,i-1) - ay*zkup(1,i-1);
            ax = (zkup(2,i)-zkup(2,i-1))/(zkup(1,i)-zkup(1,i-1));
            bx = zkup(2,i-1) - ax*zkup(1,i-1);
            ip = [bx; by]
            if k == round(length(t)/10)
                lip = [lip ip]
            elseif ip(1)>=0.95*mlip(1) && ip(1)<=1.05*mlip(1) && i>2 && ip(2)>=0.95*mlip(2) && ip(2)<=1.05*mlip(2) % j'enlève les valeurs abérantes
                lip = [lip ip]
            end
            mlip = [mean(lip(1,:)); mean(lip(2,:))]
        end
    end
end

hold off
%axis([0 z0 -0.5 0.5 0 2]);
grid on
xlabel('z(t)')
ylabel('x(t)')
zlabel('y(t)')