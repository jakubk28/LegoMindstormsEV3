% LQR controller
K1 = lqrd(A1, B1, (C1')*C1, 0.01, Ts);
% Observer using pole placement
dss = c2d(ss(A1,B1,C1,zeros(2,1)),Ts);
Lo = place(dss.A',dss.C',[0.77 0.78 0.79 0.8])';
kest = ss(dss.A-Lo*dss.C, [dss.B Lo], eye(4), zeros(4,3));
% Critically damped proportional controller for turning motion
K2 = A2(2,2)^2/(4*B2(2,1));