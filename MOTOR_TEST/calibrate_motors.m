% Measure EV3 motor parameters
load('MotorTest.mat');
figure;
xd = MotorAngles.time;
% Find average motor angle
yd = cast((1/2)*(MotorAngles.signals(1).values+MotorAngles.signals(2).values),'double');
run('../model.m');
% Convert motor angle to horizontal distance
yd = yd*pi*R/180;
% Plot the data
plot(xd, yd);
% T0 = step time. Estimate steady state velocity from least squares fit to
% horizontal distance between times T1 and T2
T0 = 1; T1 = 1.4; T2 = 1.5;
s1 = find(MotorAngles.time >= T1, 1);
s2 = find(MotorAngles.time < T2, 1, 'last');
lrx = [ones(s2 + 1 - s1,1), yd(s1:s2)];
bf = lrx\xd(s1:s2);
% Find x axis intercept (a) and gradient (b) of least squares fit to
% horizontal distance between times T1 and T2
a = bf(1,1); b = 1/bf(2,1);
hold on;
grid on;
yslbf = b*(xd - a);
% Plot steady state best fit
plot(xd, yslbf,'--');
axis([T0,max(xd),0,max(yd)]);
% Plot model prediction
ybf = b*((xd - T0) - (a-T0)*(1 - exp(-(xd - T0)/(a-T0))));
plot(xd,ybf,':k');
axis([T0,T2,0,b*(T2-T0)]);
ylabel('Distance (m)');
xlabel('Time (s)');
legend('Experimental data', 'Steady state best fit', 'Model prediction');
% Size of motor input
u = 100;
beta = R^2*(M + 2*m + (2*Jw/R^2))/(2*(a-T0));
alpha = beta*b/(R*u);
disp(['alpha = ', num2str(alpha)]);
disp(['beta = ', num2str(beta)]);