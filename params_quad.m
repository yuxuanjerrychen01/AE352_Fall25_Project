function p = params_quad()
% Physical parameters (realistic 1.5 kg class quadrotor)

p.m  = 1.5;        % mass [kg]
p.g  = 9.81;       % gravity [m/s^2]
p.L  = 0.25;       % arm length [m]

p.Jx = 0.03;       % inertia [kg m^2]
p.Jy = 0.03;
p.Jz = 0.05;
p.J  = diag([p.Jx, p.Jy, p.Jz]);

p.kT = 1.9e-5;     % thrust coeff [N s^2]
p.kQ = 2.6e-7;     % drag coeff  [N m s^2]

p.omega_max = 900; % max rotor speed [rad/s]

% Mixing matrix u = M * omega^2, u = [T; tau_phi; tau_theta; tau_psi]
kT = p.kT; kQ = p.kQ; L = p.L;
p.M = [ kT,   kT,   kT,   kT;
        0,    L*kT, 0,   -L*kT;
       -L*kT, 0,    L*kT, 0;
        kQ,  -kQ,   kQ,  -kQ ];
p.Minv = inv(p.M);

% ---------- Outer-loop gains ----------
p.kp_pos = diag([4.0, 4.0, 6.0]);   % position P (x,y,z)
p.kd_pos = diag([3.0, 3.0, 4.5]);   % velocity D

% ---------- Inner-loop gains ----------
p.kR = diag([8.0, 8.0, 4.0]);       % attitude error gain
p.kW = diag([2.5, 2.5, 1.5]);       % angular rate gain

end
