%% Physical parameters
g = 9.81;                       % acceleration due to gravity (m/sec^2)
m = 0.031;						% wheel weight (kg)
R = 0.027;                      % wheel radius (m)
                            	%0.045/2 for small, 0.07/2 for big	% wheel radius [m]
Jw = m * R^2 / 2    ;				% wheel moment of inertia (kgm^2)
M = 0.6412;                     % body mass (kg)
W = 0.105;						% body width (m)
D = 0.1;						% body depth (m)
L = 0.107;						% distance of the center of mass from the wheel axle (m)
T = 0.77;                       % Time period of oscillation, gyroboy hanging down as pendulum (s)
Jpsio = (M*g*L)/((2*pi)/T)^2;   % body pitch inertia moment about wheel axis (kgm^2)
Jpsi = Jpsio - M*L^2;           % body pitch inertia moment about c.o.g (kgm^2)
Jpsich = M * L^2 / 3;   		% body pitch inertia moment - check against above (kgm^2)
Jphi = M * (W^2 + D^2) / 12;	% body yaw inertia moment (kgm^2)
alpha = 6.73e-4;                % Motor calibration constant
beta = 0.00446;                 % Motor calibration constant

% Alternative estimates for alpha and beta (less good!)
%alpha = 0.0011387;              % motor characteristic
%beta = 0.0074492;               % other motor characteristic

%alpha = 3E-4;                   % Estimate from oscillation
%beta = alpha/0.15;              % Another estimate from oscillation

Psi0 = 0.0;                     % Initial value to make disturbance
Ts = 0.002;                     % Control system sample time
%
% Low pass filter time constant for differentiated signals
tau = 0.01;
a = 0.8;
% Gyro calibration filter
ag = 0.8;
% Model
% E = [(2*m+M)*R^2 + 2*Jw, M*L*R;
%       M*L*R, Jpsio];
% F = 2*[beta   -beta; 
%        -beta     beta];
% G = [0   0; 
%      0  -M*g*L];
% H = 2*[alpha; 
%       -alpha];
% I = (m * R^2) + (2*Jphi*R^2/W^2) + Jw;
g6 = (m * R^2) + (2*Jphi*R^2/W^2) + Jw;
% J = beta;
% K = alpha;
% Alternative approach
Jhw = Jw+ m*R^2;
g1 = M*g*L;
g2 = M*L*R;
g3 = 2*(Jw + m*R^2) + M*R^2;
g4 = M*L^2 + Jpsi;
g5 = g3*g4 - g2^2;
A1 = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, -g1*g2/g5, -2*beta*(g2+g4)/g5, 2*beta*(g2+g4)/g5;
    0, g1*g3/g5, 2*beta*(g2+g3)/g5, -2*beta*(g2+g3)/g5];
B1 = [0;0;2*alpha*(g2+g4)/g5; -2*alpha*(g2+g3)/g5];
% Pole placement formulae (all at -1 in this case)
%a3 = 4; a2 = 6; a1 = 4; a0 = 1;
%k1 = -a0*g5/(2*alpha*g1);
%k2 = -(g5*(g1*a2+(g2+g4)*a0) + g1^2*g3)/(2*alpha*g1*(g2+g3));
%k3 = (-a1*g5/(2*alpha*g1)) - beta/alpha;
%k4 = beta/alpha - (g5*(g1*a3 + (g2+g4)*a1)/(2*alpha*g1*(g2+g3)));
%% State space matrices
% A1 = [0 0   1 0;
% 	  0 0   0 1;
%      -E\G  -E\F]; 
% B1 = [0;
% 	  0;
% 	  E\H];
C1 = eye(4);
%C1o = [1,-1,0,0;
%    0,0,0,1];           %Ideal observed outputs: theta - psi and psidot
C1o = [1, -1, 0, 0;     % Use this one for ca_obs (integrating gyro sensor)
    0, 1, 0, 0];
D1 = zeros(4, 1);
% Model for integral action
A1i = [0, [1,0,0,0];
    [0;0;0;0],A1];
B1i = [0;B1];
A2 = [0   1;
	  0  -beta/g6];
B2 = [0;
    alpha/g6];
C2 = eye(2);
C2o = [1,0];                    %Ideal observed outputs: phi
D2 = zeros(2,1);
Cm = [1,-1,0,0,1,0;
     1,-1,0,0,-1,0]; % Gain for getting motor outputs: theta_r and theta_l
Cg = [0,0,0,1,0,0]; % Gain for getting gyro output
% Controllers
% LQR controller for forward motion
Q1i = eye(5);
Q1i(4,4) = 0.2;
%R1i = 2/8.087^2;
R1i = 0.01;
K1i = lqrd(A1i, B1i, Q1i, R1i, Ts);
% Or try following pole placement controller
%K1i = place(A1i,B1i,[-30,-7.5,-6.0,-0.75+0.5*j,-0.75-0.5*j]);
% Following increases overshoot and makes it fall over!
%K1i = place(A1i,B1i,[-10,-7.5,-6.0,-1.7,-1.6]);
%
% For comparison at different values of alpha and beta
%K1i = [-3.7287, -7.1966, -480.0805, -11.7650, -47.1338];
%K1i = [-2.8233, -5.7324, -668.8285, -11.0837, -55.6199];
% Critically damped proportional controller for turning motion
K2 = A2(2,2)^2/(4*B2(2,1));
% Could try following for kalman filter
%[kest,L2,P2] = kalmd(ss(A1,[B1 B1],C1o,zeros(2,2)),1/2,[1/2,0;0,1]*(pi/180)^2,Ts);
%[kest,L2,P2] = kalmd(ss(A1,[B1 B1],C1o,zeros(2,2)),0.01,[1/2,0;0,25]*(pi/180)^2,Ts);
ssd = c2d(ss(A1,B1,C1o,zeros(2,1)),Ts);
L2t = place(ssd.A',ssd.C',[0.797,0.798,0.799,0.8])';
kest = ss(ssd.A - L2t*ssd.C, [ssd.B L2t], ssd.C, zeros(2,3));
[kest,L2,P2] = kalmd(ss(A1,[B1, eye(4)],C1o,zeros(2,5)),eye(4),[1/2,0;0,1]*(pi/180)^2,Ts);