% Goal 2: Circular flight radius 2 m, speed 0.5 m/s, z=1 m for 1 min

clear; close all; clc;
p = params_quad();

R = 2.0; v = 0.5;

% Start on the circle with correct tangential velocity
x0 = zeros(12,1);
x0(1) = R; x0(2) = 0; x0(3) = 1.0;
x0(4) = 0; x0(5) = v; x0(6) = 0;

tspan = [0 60];
traj_handle = @(t) traj_circle(t,R,v);

odefun = @(t,x) quad_dynamics(t,x,p,traj_handle);
opts   = odeset('RelTol',1e-6,'AbsTol',1e-7);
[t_sol,x_sol] = ode45(odefun, tspan, x0, opts);

pos = x_sol(:,1:3);

figure;
plot3(pos(:,1),pos(:,2),pos(:,3),'LineWidth',1.6);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Goal 2: Circular trajectory (3D)');
grid on; axis equal;
saveas(gcf,'circle_3d.png');

figure;
plot(pos(:,1),pos(:,2),'LineWidth',1.6); hold on;
th = linspace(0,2*pi,200);
plot(R*cos(th),R*sin(th),'--','LineWidth',1.0);
xlabel('x [m]'); ylabel('y [m]');
title('Goal 2: XY projection'); grid on; axis equal;
legend('Drone path','Ideal circle');
saveas(gcf,'circle_xy.png');
