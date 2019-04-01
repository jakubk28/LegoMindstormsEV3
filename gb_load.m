% Model parameters for the Simulink file xb_kf_pc.slx. A controller for
% the LEGO MINDSTORMS EV3 Gyroboy model from the Education set. Based on
% the final design described in the paper 
%   Observer-based feedback controller for the LEGO MINDSTORMS EV3
%   Gyroboy segway robot, Hughes, T.H., Willetts, G.H., and Kryczka, J.A.,
%   submitted to the 2019 IEEE Conference on Decision and Control (CDC)
%
% Relative to the final design described in that paper, the Kalman filter
% has been slightly modified to improve stability, a pre-compensator
% has been added to the reference input to cause a smoother motion, and
% high pass filters have been added to the gyro sensor and the estimate of
% tilt angle to reduce issues with gyro drift and gyro calibration error.
%% Gyroboy model configuration
real_model = 1; % 1 models quantisation of sensors
HW = 0; % 1 is for running model on EV3, 0 for running in simulation mode.
%% Physical parameters
g = 9.81;                       % acceleration due to gravity (m/sec^2)
m = 0.031;						% wheel weight (kg)
R = 0.027;                      % wheel radius (m)
                            	%0.045/2 for small, 0.07/2 for big	% wheel radius [m]
Jw = m * R^2 / 2;				% wheel moment of inertia (kgm^2)
M = 0.6412;                     % body mass (kg)
W = 0.105;						% body width (m)
D = 0.1;						% body depth (m)
L = 0.107;						% distance of the center of mass from the wheel axle (m)
T = 0.77;                       % Time period of oscillation, gyroboy hanging down as pendulum (s)
Jpsio = (M*g*L)/((2*pi)/T)^2;   % body pitch inertia moment about wheel axis (kgm^2)
Jpsi = Jpsio - M*L^2;           % body pitch inertia moment about c.o.g (kgm^2)
Jphi = M * (W^2 + D^2) / 12;	% body yaw inertia moment (kgm^2)
alpha = 6.73e-4;                % Motor calibration constant
beta = 0.00446;                 % Motor calibration constant
%
%% Controller design
Ts = 0.002;                     % Control system sample time
%
% Low pass filter time constant for differentiated signals
a = 0.8;
% Gyro calibration filter
ag = 0.8;
% Gyro high pass filter
tau_h = 10;
ah = (tau_h/Ts)/(1 + (tau_h/Ts));
% Model linearised about upright equilibrium
Jhw = Jw+ m*R^2;
g1 = M*g*L;
g2 = M*L*R;
g3 = 2*(Jw + m*R^2) + M*R^2;
g4 = M*L^2 + Jpsi;
g5 = g3*g4 - g2^2;
g6 = (m * R^2) + (2*Jphi*R^2/W^2) + Jw;
A1 = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, -g1*g2/g5, -2*beta*(g2+g4)/g5, 2*beta*(g2+g4)/g5;
    0, g1*g3/g5, 2*beta*(g2+g3)/g5, -2*beta*(g2+g3)/g5];
B1 = [0;0;2*alpha*(g2+g4)/g5; -2*alpha*(g2+g3)/g5];
C1 = eye(4);        % Used to obtain forward states in simulation
C1o = [1, -1, 0, 0; % Observed forward outputs
    0, 1, 0, 0];
D1 = zeros(4, 1);
% Model for integral action
A1i = [0, [1,0,0,0];
    [0;0;0;0],A1];
B1i = [0;B1];
% Turning motion dynamics
A2 = [0   1;
	  0  -beta/g6];
B2 = [0;
    alpha/g6];
C2 = eye(2);               %Used to obtain turning states in simulation            
C2o = [1,0];               %Observed turning outputs
D2 = zeros(2,1);
Cm = [1,-1,0,0,1,0;
     1,-1,0,0,-1,0];    % Gain for getting motor outputs: theta_r and theta_l
Cg = [0,0,0,1,0,0];     % Gain for getting gyro output
% Controllers
% LQR controller for forward motion
Q1i = eye(5);
R1i = 0.01;
K1i = lqrd(A1i, B1i, Q1i, R1i, Ts);
% Critically damped proportional controller for turning motion
K2 = A2(2,2)^2/(4*B2(2,1));
% Kalman filter
[kest,L2,P2] = kalmd(ss(A1,[B1, eye(4)],C1o,zeros(2,5)),eye(4),eye(2)*1E-6,Ts);
% Precompensator design
load sysnss.mat;