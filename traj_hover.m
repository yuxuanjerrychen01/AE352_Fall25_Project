function ref = traj_hover(t)
% pr, vr, ar, psir for hover
pr   = [0;0;1];
vr   = [0;0;0];
ar   = [0;0;0];
psir = 0;
ref = [pr; vr; ar; psir];
end
