% Goal 1: Hover 1 m above ground for 2 minutes

clear; close all; clc;
p = params_quad();

x0 = zeros(12,1);
x0(3) = 0.2;               % start slightly below desired altitude
x0(7) = deg2rad(3);        % small roll perturbation
x0(8) = deg2rad(-2);       % small pitch perturbation

tspan = [0 120];
traj_handle = @traj_hover;

odefun = @(t,x) quad_dynamics(t,x,p,traj_handle);
opts   = odeset('RelTol',1e-6,'AbsTol',1e-7);
[t_sol,x_sol] = ode45(odefun, tspan, x0, opts);

pos = x_sol(:,1:3);
eta = x_sol(:,7:9);

figure;
plot(t_sol,pos(:,3),'LineWidth',1.6);
xlabel('Time [s]'); ylabel('z [m]');
title('Goal 1: Hover altitude'); grid on;
saveas(gcf,'hover_altitude.png');

figure;
plot(t_sol,rad2deg(eta(:,1)),'LineWidth',1.4); hold on;
plot(t_sol,rad2deg(eta(:,2)),'LineWidth',1.4);
plot(t_sol,rad2deg(eta(:,3)),'LineWidth',1.4);
xlabel('Time [s]'); ylabel('Euler angles [deg]');
legend('\phi','\theta','\psi');
title('Goal 1: Hover attitude'); grid on;
saveas(gcf,'hover_attitude.png');
