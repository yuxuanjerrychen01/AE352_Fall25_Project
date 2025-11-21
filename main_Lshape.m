% Goal 3: L-shaped mission + yaw + soft landing

clear; close all; clc;
p = params_quad();

x0 = zeros(12,1); % start at origin on ground

tspan = [0 134];  % matches total mission time
traj_handle = @traj_Lshape;

odefun = @(t,x) quad_dynamics(t,x,p,traj_handle);
opts   = odeset('RelTol',1e-6,'AbsTol',1e-7);
[t_sol,x_sol] = ode45(odefun, tspan, x0, opts);

pos = x_sol(:,1:3);
eta = x_sol(:,7:9);

figure;
plot3(pos(:,1),pos(:,2),pos(:,3),'LineWidth',1.6);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Goal 3: L-shaped mission (3D)');
grid on; axis equal;
saveas(gcf,'Lshape_3d.png');

figure;
plot(t_sol,pos(:,3),'LineWidth',1.6);
xlabel('Time [s]'); ylabel('z [m]');
title('Goal 3: Altitude profile');
grid on;
saveas(gcf,'Lshape_altitude.png');

figure;
plot(t_sol,rad2deg(eta(:,3)),'LineWidth',1.6);
xlabel('Time [s]'); ylabel('Yaw \psi [deg]');
title('Goal 3: Yaw profile');
grid on;
saveas(gcf,'Lshape_yaw.png');

% Optional: verify landing speed
vz = x_sol(:,6);
fprintf('Max |vz| during final descent = %.4f m/s\n', max(abs(vz(end-200:end))));
