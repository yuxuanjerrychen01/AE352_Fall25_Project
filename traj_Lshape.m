function ref = traj_Lshape(t)
% Goal 3 piecewise mission:
% 1) ascend to 1 m
% 2) +x 5 m at 1 m/s
% 3) hover
% 4) yaw +90 deg
% 5) +y 5 m at 1 m/s
% 6) hover
% 7) descend at <= 0.01 m/s

T1 = 4;    % ascent
T2 = 5;    % x leg (5m @1m/s)
T3 = 3;    % hover
T4 = 4;    % yaw
T5 = 5;    % y leg
T6 = 3;    % hover
T7 = 110;  % descent (1m /110s = 0.0091 m/s)

Ttot = T1+T2+T3+T4+T5+T6+T7;
t = min(t,Ttot);

pr=zeros(3,1); vr=zeros(3,1); ar=zeros(3,1); psir=0;

if t<=T1
    tau=t/T1;
    pr=[0;0; tau*1.0];
    vr=[0;0; 1.0/T1];
    ar=[0;0; 0];
    psir=0;

elseif t<=T1+T2
    tau=(t-T1)/T2;
    pr=[5.0*tau; 0; 1.0];
    vr=[5.0/T2; 0; 0];
    ar=[0;0;0];
    psir=0;

elseif t<=T1+T2+T3
    pr=[5.0;0;1.0]; vr=[0;0;0]; ar=[0;0;0]; psir=0;

elseif t<=T1+T2+T3+T4
    tau=(t-(T1+T2+T3))/T4;
    pr=[5.0;0;1.0];
    vr=[0;0;0]; ar=[0;0;0];
    psir=tau*(pi/2);

elseif t<=T1+T2+T3+T4+T5
    tau=(t-(T1+T2+T3+T4))/T5;
    pr=[5.0; 5.0*tau; 1.0];
    vr=[0; 5.0/T5; 0];
    ar=[0;0;0];
    psir=pi/2;

elseif t<=T1+T2+T3+T4+T5+T6
    pr=[5.0;5.0;1.0]; vr=[0;0;0]; ar=[0;0;0]; psir=pi/2;

else
    tau=(t-(T1+T2+T3+T4+T5+T6))/T7;
    pr=[5.0;5.0; 1.0*(1-tau)];
    vr=[0;0; -1.0/T7];          % <= 0.01 m/s
    ar=[0;0; 0];
    psir=pi/2;
end

ref=[pr; vr; ar; psir];
end
