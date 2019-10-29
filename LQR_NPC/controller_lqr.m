% LQR controller
K1 = lqrd(A1, B1, (C1')*C1, 0.01, Ts);
% Critically damped proportional controller for turning motion
K2 = A2(2,2)^2/(4*B2(2,1));