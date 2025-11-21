function ref = traj_circle(t, R, v)
% Circle in xy-plane at altitude 1 m
omega = v/R;

pr = [ R*cos(omega*t);
       R*sin(omega*t);
       1.0 ];

vr = [ -R*omega*sin(omega*t);
        R*omega*cos(omega*t);
        0.0 ];

ar = [ -R*omega^2*cos(omega*t);
       -R*omega^2*sin(omega*t);
        0.0 ];

psir = 0; % fixed yaw

ref = [pr; vr; ar; psir];
end
