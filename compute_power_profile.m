function [P_tot, E_Wh, SOC] = compute_power_profile(t, x, p, traj_handle)
%COMPUTE_POWER_PROFILE Approximate total mechanical power and battery usage.
%   t           : time vector from ODE solver (Nx1)
%   x           : state trajectory (Nx12)
%   p           : parameter struct (needs kp_pos, kd_pos, kR, kW, etc.)
%   traj_handle : same reference generator used in simulation
%
%   P_tot [W]   : approximate total mechanical power vs time
%   E_Wh [Wh]   : integrated energy vs time
%   SOC [-]     : estimated battery state-of-charge vs time (1 = full)

m   = p.m;
g   = p.g;
J   = p.J;
e3  = [0;0;1];

N = numel(t);
P_tot = zeros(N,1);

for k = 1:N
    tk = t(k);
    state = x(k,:).';
    
    pos   = state(1:3);
    vel   = state(4:6);
    eta   = state(7:9);     % [phi; theta; psi]
    omega = state(10:12);   % [p; q; r]
    
    phi   = eta(1); 
    theta = eta(2); 
    psi   = eta(3);
    
    % Rotation matrix (body -> inertial)
    R = rotm_zyx(phi, theta, psi);
    
    % Reference from trajectory
    ref = traj_handle(tk);
    pr   = ref(1:3);
    vr   = ref(4:6);
    ar   = ref(7:9);
    psir = ref(10);
    
    % --- Outer-loop: desired acceleration (same as in dynamics) ---
    ep = pr - pos;
    ev = vr - vel;
    
    a_des = ar + p.kp_pos*ep + p.kd_pos*ev;
    f_des = m*(a_des + g*e3);
    
    % --- Desired attitude R_des (same construction as in dynamics) ---
    zb_des = f_des / max(1e-6, norm(f_des));
    xc_des = [cos(psir); sin(psir); 0];
    yb_des = cross(zb_des, xc_des);
    if norm(yb_des) < 1e-6
        yb_des = [0;1;0];  % fallback
    else
        yb_des = yb_des / norm(yb_des);
    end
    xb_des = cross(yb_des, zb_des);
    R_des  = [xb_des, yb_des, zb_des];
    
    % --- Thrust command ---
    b3   = R*e3;
    Tdes = dot(f_des, b3);
    Tdes = max(0, Tdes);
    
    % --- Attitude error and torque command ---
    E_mat = 0.5*(R_des.'*R - R.'*R_des);
    e_R   = [E_mat(3,2); E_mat(1,3); E_mat(2,1)];
    
    omega_des = [0;0;0];
    e_W       = omega - R.'*R_des*omega_des;
    
    tau = -p.kR*e_R - p.kW*e_W + cross(omega, J*omega);
    
    % --- Motor mixing to get rotor speeds ---
    u = [Tdes; tau];
    
    omega_sq = p.Minv * u;
    omega_sq = max(0, omega_sq);
    omega_vec = sqrt(omega_sq);
    omega_vec = min(omega_vec, p.omega_max);
    
    % --- Approximate mechanical power: sum(T_i * omega_i) ---
    P_tot(k) = p.kT * sum(omega_vec.^3);
end

% Integrate power to get energy [J] and convert to [Wh]
E_J  = cumtrapz(t, P_tot);    % [J]
E_Wh = E_J / 3600;            % [Wh]

% Battery state-of-charge (if battery info available)
if isfield(p, 'Ebat_Wh')
    SOC = max(0, 1 - E_Wh / p.Ebat_Wh);
else
    SOC = nan(size(E_Wh));
end

end

% ----- helper rotation function -----
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
