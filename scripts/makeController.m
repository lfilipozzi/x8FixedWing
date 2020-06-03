function makeController(varargin)
% MAKECONTROLLER Design the controller.
% Argument
%   'find_jacobian', values: 'true'

find_jacobian = false;

i = 1;
while i <= nargin
    if strcmpi(varargin{i},'find jacobian')
        if i+1 > nargin
            error('Argument must correspond to key + value, Missing key or value.')
        end
        if ~ismember(varargin{i+1},{'true','false'})
            error('Incorrect value for argument "find jacobian"')
        end
        find_jacobian = eval(varargin{i+1});
        i = i+1;
    end
    i = i+1;
end

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
]; %#ok<NASGU>

u0 = [...
    0.0370;
   -0.0000;
         0;
    0.1219;
]; %#ok<NASGU>

if find_jacobian
    disp('Recompute Jacobian matrices')
    parameters;
    % Find equilibrium
    options = optimoptions('fsolve');
    options.Display = 'none';
    options.Algorithm = 'levenberg-marquardt';
    X = fsolve(...
        @(xu) dynamics(xu(1:Nx),xu(Nx+(1:Nu)),zeros(6,1),P),...
        [x0;u0],...
        options...
    );
    x0 = X(1:Nx);
    u0 = X(Nx+(1:Nu));
    % Compute Jacobian matrices
    [jac,~] = jacobianest(...
        @(xu) dynamics(xu(1:Nx),xu(Nx+(1:Nu)),zeros(6,1),P),...
        [x0;u0]...
    );
    A = jac(1:Nx,1:Nx);
    B = jac(1:Nx,1:Nu);
else
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
end

C = [...
    0 0 0 0 0 0 1 0 0 0 0 0;    % Airspeed
    0 0 0 1 0 0 0 0 0 0 0 0;    % Roll angle
    0 0 0 0 1 0 0 0 0 0 0 0;    % Pitch angle
];

D = zeros(Nr,Nu);

% Reduce system
sys = ss(A,B,C,D);
[sysr,T] = minreal(sys);
NxRed = size(sysr.A,1);

% LQR synthesis
r = 10;
Q = T\diag(ones(Nx,1))*T;
Q = Q(1:NxRed,1:NxRed);
Q = blkdiag(Q,diag(ones(Nr,1)));
R = r*diag(ones(Nu,1));
N = zeros(NxRed+Nr,Nu);
[K,S,e] = lqi(sysr,Q,R,N); %#ok<ASGLU>

save('data/controller');

end