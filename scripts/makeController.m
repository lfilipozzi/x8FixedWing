% function ctrl = makeController()
% MAKECONTROLLER Design the controller.

Nx = 12;    % Number of states
Nu = 4;     % Number of inputs
Nr = 3;     % Number of targets

x0 = [...
   -0.0000;
    0.0000;
 -200.0000;
   -0.0000;
    0.0308;
   -0.0000;
   17.9914;
   -0.0000;
    0.5551;
   -0.0000;
   -0.0000;
   -0.0000;
];

u0 = [...
    0.0370;
   -0.0000;
         0;
    0.1219;
];

A = [...
         0         0         0         0         0         0    0.9995         0    0.0308         0         0         0;
         0         0         0   -0.5551         0   18.0000   -0.0000    1.0000   -0.0000         0         0         0;
         0         0         0         0  -18.0000         0   -0.0308         0    0.9995         0         0         0;
         0         0         0   -0.0000   -0.0000         0         0         0         0    1.0000   -0.0000    0.0309;
         0         0         0    0.0000         0         0         0         0         0         0    1.0000    0.0000;
         0         0         0   -0.0000   -0.0000         0         0         0         0         0   -0.0000    1.0005;
         0         0         0         0   -9.8053         0   -0.0990    0.0144    0.4924         0   -0.5027         0;
         0         0         0    9.8053    0.0000         0    0.0000   -0.4932   -0.0000    0.2005         0  -17.7750;
         0         0         0         0   -0.3025         0   -0.7828    0.0004   -9.9574    0.0000   16.2936         0;
         0         0         0         0         0         0   -0.0000   -4.0047   -0.0000  -30.6162   -0.0000   -1.5978;
         0         0         0         0         0         0    0.2477         0   -8.0279         0   -4.0317         0;
         0         0         0         0         0         0   -0.0000   -3.6900   -0.0000  -32.3855   -0.0000   -3.1853;
];
         
B = [...
         0         0         0         0;
         0         0         0         0;
         0         0         0         0;
         0         0         0         0;
         0         0         0         0;
         0         0         0         0;
    0.1722         0         0    9.5266;
   -0.0000    1.9147         0         0;
  -12.3037         0         0         0;
         0  153.1492         0         0;
  -71.5829         0         0         0;
         0  161.2486         0         0;
];

C = [...
    0 0 0 0 0 0 1 0 0 0 0 0;    % Airspeed
    0 0 0 1 0 0 0 0 0 0 0 0;    % Roll angle
    0 0 0 0 1 0 0 0 0 0 0 0;    % Pitch angle
];

D = zeros(Nr,Nu);

% % LQR synthesis
% r = 10;
% Q = diag(ones(Nx,1));
% R = r*diag(ones(Nu,1));
% [K,S,e] = lqr(A,B,Q,R);

% % LQR synthesis
% r = 10;
% Q = diag(ones(Nx,1));
% R = r*diag(ones(Nu,1));
% % Add integral action
% A = [A zeros(Nx,Nr); -C zeros(Nr)];
% B = [B; -D];
% Q = blkdiag(Q,diag(ones(Nr,1)));
% [K,S,e] = lqr(A,B,Q,R);

% LQR synthesis
sys = ss(A,B,C,D);
[sysr,T] = minreal(sys);
NxRed = size(sysr.A,1);
r = 10;
Q = T\diag(ones(Nx,1))*T;
Q = Q(1:NxRed,1:NxRed);
Q = blkdiag(Q,diag(ones(Nr,1)));
R = r*diag(ones(Nu,1));
N = zeros(NxRed+Nr,Nu);
[K,S,e] = lqi(sysr,Q,R,N);

% % LQR synthesis
% r = 10;
% Q = diag(ones(Nx+Nr,1));
% R = r*diag(ones(Nu,1));
% N = zeros(Nx+Nr,Nu);
% sys = ss(A,B,C,D);
% [K,S,e] = lqi(sys,Q,R,N);

save('data/controller');

% end