% PARAMETERS Load system parameters

P = load('x8_param.mat');

% Inertia matrix: assuming symmetry wrt xz-plane -> Jxy=Jyz=0
P.I_cg =  [P.Jx,  0,  -P.Jxz;
         0      P.Jy,    0;
         -P.Jxz,0,    P.Jz];
% Mass matrix
P.M_rb = [eye(3)*P.mass,     -P.mass*Smtrx(P.r_cg);
              P.mass*Smtrx(P.r_cg),      P.I_cg       ];
P.rho = 1.2250;
P.gravity = 9.81;

% Sampling time
P.Ts = 0.001;