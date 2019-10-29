% Discrete approximation to high pass filter with time constant tau_h
tau_h = 0.1;
ah = (tau_h/Ts)/(1 + (tau_h/Ts));