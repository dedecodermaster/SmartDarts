function xfinal = pred_traj3D(mesuresZ,mesuresX,mesuresY) %#codegen
    arguments
        mesuresZ (1,:) double
        mesuresX (1,:) double
        mesuresY (1,:) double
    end


    deltat = 1/120; % Fréquence des caméras
    t = 0:deltat:0.15;
    
    lip = zeros(2,0);
    mlip = [3;0.5];
    g = 9.81;
    
    zkinit = zeros(6,1);
    ykinit = zeros(3,length(mesuresZ));
    xfinal = zeros(6,length(t));
    yk = zeros(3,length(t));
    zk = zeros(6,length(t));
    
    % Prédiction voulue au départ = conditions initiales
    zkinit(1,1) = mesuresZ(1);
    zkinit(2,1) = mesuresX(1);
    zkinit(3,1) = mesuresY(1);
    zkinit(4,1) = ((mesuresZ(2)-mesuresZ(1))/deltat);
    zkinit(5,1) = ((mesuresX(2)-mesuresX(1))/deltat);
    zkinit(6,1) = ((mesuresY(2)-mesuresY(1))/deltat)+0.04;
    
   
    Gzk = cov(zkinit)*10;
    uk = [0; 0; 0; 0; 0; 0]; % Entrée du système (pour l'instant aucune force extérieures
    Ak = [1 0 0 deltat 0 0; 0 1 0 0 deltat 0; 0 0 1 0 0 deltat*(1-(1/2)*9.81);0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]; % Matrice de passage entre l'état actuel et l'état suivant
    Galphak = [0.0015 0 0 0 0 0; 0 0.0015 0 0 0 0; 0 0 0.0015 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]; % Précision de la caméra
    Ck = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]; % On ne mesure que les positions
    Gbetak = [0 0 0; 0 0 0; 0 0 0]; % on autorise une erreur de 5 millimètres selon les deux axes, sinon on a peu de prédiction
    
    for i = 1:length(mesuresZ)
        ykinit(1,i) = mesuresZ(i);
        ykinit(2,i) = mesuresX(i);
        ykinit(3,i) = mesuresY(i);
        yk(1,i) = ykinit(1,i);
        yk(2,i) = ykinit(2,i);
        yk(3,i) = ykinit(3,i);
    end

    [zk,Gzk,zkup,Gzkup] = testKalman(zkinit,Gzk,uk,ykinit,Galphak,Gbetak,Ak,Ck);

    for r = size(zkup,2)
            xfinal(:,r)= zkup(:,r);
    end
    "sortie";
    for j = length(mesuresZ):length(t)-1 % Création d'une trajectoire simulée
        yc = zkup(6,1)*t(j+1:length(t))-(1/2)*g*(t(j+1:length(t)).^2) + yk(3,1);
        zc = yk(1,1) + zkup(4,1)*t(j+1:length(t));
        xc = yk(2,1) + zkup(5,1)*t(j+1:length(t));
        yk(1,j:length(t)-1) = zc; % Définition de la trajectoire simulée comme la mesure
        yk(2,j:length(t)-1) = xc;
        yk(3,j:length(t)-1) = yc;
        for r = 1:j
            xfinal(:,r) = zkup(:,r);
        end
        xfinal(:,j+1) = zk(:,j);
%         zk = [zkup zk(:,j)]; 
        [zk,Gzk,zkup,Gzkup] = testKalman(xfinal(:,1:j+1),Gzk,uk,yk(:,1:j+1),Galphak,Gbetak,Ak,Ck)
    end
end

