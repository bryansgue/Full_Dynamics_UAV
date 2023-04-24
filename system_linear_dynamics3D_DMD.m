function [h] = system_linear_dynamics3D_DMD(h, T, A, B, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

k1 = linear_dynamics3D_DMD(h, T, A, B);
k2 = linear_dynamics3D_DMD(h + ts/2*k1, T, A, B); % new
k3 = linear_dynamics3D_DMD(h + ts/2*k2, T, A, B); % new
k4 = linear_dynamics3D_DMD(h + ts*k3,  T, A, B); % new

h = h +ts/6*(k1 +2*k2 +2*k3 +k4); % new
end
