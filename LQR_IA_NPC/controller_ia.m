% Model for integral action
A1i = [0, [1,0,0,0];
    [0;0;0;0],A1];
B1i = [0;B1];
C1i = [1 zeros(1,4); zeros(2,1) C1];
% LQR controller with integral action
K1 = lqrd(A1i, B1i, C1i'*C1i, 0.01, Ts);
% Critically damped proportional controller for turning motion
K2 = A2(2,2)^2/(4*B2(2,1));