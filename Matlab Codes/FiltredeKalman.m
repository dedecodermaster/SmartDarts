function xest = FiltredeKalman(z,y,deltat)
    % Définition du modèle
    dF = [0 0 1 0; 0 0 0 1-(1/2)*9.81;0 0 0 0;0 0 0 0]; % matrice du système linéaire d'eq. diffs

    F = expm(dF*deltat); % matrice du modèle discret

    sigma_q = 1;
    Q = sigma_q^2 %* [0 0 0 0;0 0 0 0;0 0 1 0;0 0 0 1];

    H = [1 0 0 0; 0 1 0 0]; % Matrice de mesure de la position uniquement

    sigma_r = 1;
    R = eye(2) * sigma_r^2;

    B = [0; 0; 0; 0]

    sys = ss(F,B,H,[0;0]);

    % Initialisation
    X0 = [z(1); y(1); 0 ; 0]; % première estimation de l'état
    P0 = diag([sigma_r sigma_r 100 100]); % covariance de l'estimation
    %X = [z;y]

    % Filtre
    [xest, Pest, K, xap] = kalman(sys, Q, R); % Kalman
end
