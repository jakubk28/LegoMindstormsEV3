%% Physical parameters
g = 9.81;                       % acceleration due to gravity (m/sec^2)
m = 0.031;						% wheel weight (kg)
R = 0.027;                      % wheel radius (m)
Jw = m * R^2 / 2;				% wheel moment of inertia (kgm^2)
M = 0.641;                      % body mass (kg)
W = 0.105;						% body width (m)
D = 0.1;						% body depth (m)
L = 0.107;						% distance of the center of mass from the wheel axle (m)
T = 0.77;                       % Time period of oscillation, gyroboy hanging down as pendulum (s)
Jpsio = (M*g*L)/((2*pi)/T)^2;   % body pitch inertia moment about wheel axis (kgm^2)
Jpsi = Jpsio - M*L^2;           % body pitch inertia moment about c.o.g (kgm^2)
Jphi = M * (W^2 + D^2) / 12;	% body yaw inertia moment (kgm^2)
alpha = 5.05e-4;                % Motor calibration constant
beta = 0.00358;                 % Motor calibration constant
Ts = 0.002;                     % Sample time
%
% Low pass filter time constant for signals to be differentiated, with time
% constant tau.
tau = 4*Ts;
a = (tau/Ts)/(1 + (tau/Ts));
% Gyro calibration filter
ag = a;
% Gyro high pass filter: discrete approximation to high pass filter with 
% time constant tau_h
tau_h = 10;
ah = (tau_h/Ts)/(1 + (tau_h/Ts));
% Forward dynamics linearised about upright equilibrium
Jhw = Jw+ m*R^2;
g1 = M*g*L;
g2 = M*L*R;
g3 = 2*(Jw + m*R^2) + M*R^2;
g4 = M*L^2 + Jpsi;
g5 = g3*g4 - g2^2;
g6 = (m * R^2) + (2*Jphi*R^2/(W^2)) + Jw;
A1 = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, -g1*g2/g5, -2*beta*(g2+g4)/g5, 2*beta*(g2+g4)/g5;
    0, g1*g3/g5, 2*beta*(g2+g3)/g5, -2*beta*(g2+g3)/g5];
B1 = [0;0;2*alpha*(g2+g4)/g5; -2*alpha*(g2+g3)/g5];
C1 = [1, -1, 0, 0;
    0, 1, 0, 0];    % Measured forward outputs = C1 x_1 + D1 u_1
% Turning motion dynamics
A2 = [0   1;
	  0  -beta/g6];
B2 = [0;
    2*R*alpha/(W*g6)];
C2 = [1,0];          %Observed turning outputs = C2 x_2 + D2 u_2
Cm = [1,-1,0,0,W/(2*R),0;
     1,-1,0,0,-W/(2*R),0]; % Gain for getting motor outputs: theta_r and theta_l
Cg = [0,0,0,1,0,0];        % Gain for getting gyro output