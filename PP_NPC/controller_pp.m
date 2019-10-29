% Pole placement controller
K1 = place(A1, B1, [-30, -7.9, -5.5, -1.4]);
% Critically damped proportional controller for turning motion
K2 = A2(2,2)^2/(4*B2(2,1));