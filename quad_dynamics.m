function dx = quad_dynamics(t, x, p, traj_handle)
% Nonlinear 6-DoF quadrotor + SE(3)-style cascaded control.
% x = [pos(3); vel(3); eulerZYX(3); omega_body(3)].

m = p.m; g = p.g; J = p.J;
e3 = [0;0;1];

% -------- Unpack state --------
pos   = x(1:3);
vel   = x(4:6);
eta   = x(7:9);     % [phi; theta; psi]
omega = x(10:12);   % body rates [p;q;r]

phi=eta(1); theta=eta(2); psi=eta(3);

% Rotation matrix R (body -> inertial)
R = rotm_zyx(phi, theta, psi);

% -------- Reference --------
ref = traj_handle(t);
pr   = ref(1:3);
vr   = ref(4:6);
ar   = ref(7:9);
psir = ref(10);

% -------- Outer-loop: desired acceleration --------
ep = pr - pos;
ev = vr - vel;

a_des = ar + p.kp_pos*ep + p.kd_pos*ev;  % desired inertial accel

% Desired total force (thrust vector in inertial frame)
f_des = m*(a_des + g*e3);

% -------- Build desired attitude R_des --------
% Desired body z-axis aligned with thrust vector
zb_des = f_des / max(1e-6, norm(f_des));

% Desired yaw defines desired body x-axis projection
xc_des = [cos(psir); sin(psir); 0];
yb_des = cross(zb_des, xc_des);
if norm(yb_des) < 1e-6
    yb_des = [0;1;0]; % fallback (rare)
else
    yb_des = yb_des / norm(yb_des);
end
xb_des = cross(yb_des, zb_des);

R_des = [xb_des, yb_des, zb_des];

% -------- Thrust command --------
% Project desired thrust onto current body z-axis
b3 = R*e3;
Tdes = dot(f_des, b3);
Tdes = max(0, Tdes);

% -------- Inner-loop: attitude control on SO(3) --------
% Rotation error e_R = 0.5 vee(R_des^T R - R^T R_des)
E = 0.5*(R_des'*R - R'*R_des);
e_R = [E(3,2); E(1,3); E(2,1)];

% Desired angular velocity = 0 (tracking slowly varying R_des)
omega_des = [0;0;0];
e_W = omega - R'*R_des*omega_des;

% PD + gyro compensation
tau = -p.kR*e_R - p.kW*e_W + cross(omega, J*omega);

u = [Tdes; tau];

% -------- Mixing to rotors --------
omega_sq = p.Minv*u;
omega_sq = max(0, omega_sq);
omega_vec = sqrt(omega_sq);
omega_vec = min(omega_vec, p.omega_max); % saturate

% Recompute actual thrust/torques after saturation
T_total = p.kT*sum(omega_vec.^2);
tau_vec = [ ...
    p.L*p.kT*(omega_vec(2)^2 - omega_vec(4)^2);
    p.L*p.kT*(omega_vec(3)^2 - omega_vec(1)^2);
    p.kQ*(omega_vec(1)^2 - omega_vec(2)^2 + ...
          omega_vec(3)^2 - omega_vec(4)^2) ];

% -------- True dynamics --------
% IMPORTANT FIX: thrust acts upward along +b3 in inertial frame
F = -m*g*e3 + T_total*b3;

omega_dot = J \ (tau_vec - cross(omega, J*omega));

W = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
     0 cos(phi)           -sin(phi);
     0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
eta_dot = W*omega;

dx = zeros(12,1);
dx(1:3)   = vel;
dx(4:6)   = F/m;
dx(7:9)   = eta_dot;
dx(10:12) = omega_dot;
end

% ===== Helper: rotation ZYX =====
function R = rotm_zyx(phi, theta, psi)
Rz = [cos(psi) -sin(psi) 0;
      sin(psi)  cos(psi) 0;
      0         0        1];
Ry = [cos(theta) 0 sin(theta);
      0          1 0;
     -sin(theta) 0 cos(theta)];
Rx = [1 0 0;
      0 cos(phi) -sin(phi);
      0 sin(phi)  cos(phi)];
R = Rz*Ry*Rx;
end
