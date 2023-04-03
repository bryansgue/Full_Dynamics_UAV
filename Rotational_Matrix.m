%% Dynamic Model Drone

%% clear variables
clc, clear all, close all;

%% Initial conditions system
 euler = (pi/180)*[50, 41, 10];
 
 
 [R_manual] = Rot_zyx(euler)
 
 
 
 R_matlab_2 = eul2rotm(euler,'XYZ')
 