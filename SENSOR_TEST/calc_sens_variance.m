% Estimate gyroscopic sensor noise
load SensorTestResFiltered;
run('../model.m')
% Calculate variance of filtered data from 10s onwards
Tstart = 10;
st = find(GyroSensorFiltered.time >= Tstart, 1);
% End time
Tf = GyroSensorFiltered.time(end);
xd = GyroSensorFiltered.time(st:end);
yd = GyroSensorFiltered.signals.values(st:end);
% Calculate (1/T)(E(int_0^T(yd^2(t))dt))
var = (1/(Tf+Ts-Tstart))*(trapz(xd,yd.^2));
disp(['Power spectral density estimate = ', num2str(var)]);
% Plot gyroscopic sensor data
figure;
plot(GyroSensorFiltered.time(st:end),GyroSensorFiltered.signals.values(st:end));
grid on;
xlabel('Time (s)','Interpreter','latex');
ylabel('Filtered $\psi$ (degrees)','Interpreter','latex');