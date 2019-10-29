% Design precompensator to place all poles at -4 and cancel all left half
% plane zeros.
Apcd = [(A1-B1*K1) (B1*K1*[1;zeros(3,1)]); zeros(1,5)];
Bpcd = [(B1*K1*[0;0;1;0]);1];
Cpcd = [0 0 1 0 0];
Dpcd = 0;
zsys = zpk(minreal(ss(Apcd,Bpcd,Cpcd,Dpcd)));
tfp = zsys.P{1};
tfz = zsys.Z{1};
tfk = zsys.K;
np = size(tfp,1);
nz = size(tfz,1);
tfzr = tfp;
tfpr = [];
for i = 1:nz
    if real(tfz(i) < 0)
        tfpr = [tfpr; tfz(i)];
    else
        tfpr = [tfpr; -4];
    end
end
for i = nz+1:np
    tfpr = [tfpr; -4];
end
tfkr = 1;
for i = 1:np
    tfkr = (tfkr*tfpr(i,1))/tfzr(i,1);
end
cpc = ss(zpk(tfzr,tfpr,tfkr));
sysnss = c2d(cpc,Ts);