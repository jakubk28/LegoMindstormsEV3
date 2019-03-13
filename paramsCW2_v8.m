
n = 5;
alpha = 0.01;
beta = 0.01;
J = 0.2;
g = 9.81;
b = (n^2) * beta;
k = n * alpha;
Ts = 0.004;
m = 0.05;
ze = 0.2;
 
A = [0 0 1 0;0 0 0 1;0 -g/1.8 0 0;-(m*g)/(J+(m*(ze)^2)) 0 0 -b/(J+(m*(ze^2)))];
B = [0;0;0;k/(J+(m*(ze^2)))];
%A = [0 0 1 0;0 0 0 1;0 -g/1.8 0 0;(m*g)/(J) 0 0 -b/(J)];
%B = [0;0;0;k/(J)];

p1 = -1;
p2 = -1.2;
p3 = -1.4;
p4 = -1.6;

%K = place(A,B,[p1,p2,p3,p4]);

Q = eye(4);
%change Q(1,1) to be higher to increase penalty
Q(1,1) = 200;
%R = eye(1);
R = 0.01;
%K = lqr(A,B,Q,R);
%K = [-10 K];
K = lqr([0 1 0 0 0;[0;0;0;0] A],[0;B],[1 0 0 0 0; [0;0;0;0] Q],R);
%alternatively...
%K = lqrd(A,B,Q,R,Ts);

C = [1 0 0 0;0 1 0 0];

q1 = -2;
q2 = -2.2;
q3 = -2.4;
q4 = -2.6;


L = place(A',C',[q1,q2,q3,q4])';

V = 1 * [0.001^2 0;0 (pi/(180*n))^2];
W = 1 * (14/100)^2;

%[sso,L,P] = kalman(ss(A,[B B],C,zeros(2,2)),W,V);
[sso,L,P] = kalmd(ss(A,[B B],C,zeros(2,2)),W,V,Ts);
