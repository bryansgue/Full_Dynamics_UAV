%% Vertical dynamics drone

%% clean variables 
clc, clear all, close all;

%% Time definition
load('MatrixUAV_A_B_12.mat')
t_s = 0.01;
t_final = 5;
t = (0:t_s:t_final);
%% Intitial conditions position
x = 0;
y = 0;
z = 0;

ux = 0;
uy = 0;
uz = 0;

phi = 0;
theta = 0;
psi = 0;

h=[x;y;z]
s(:,1) = [ux;uy;uz;phi;theta;psi];
%% Initial condition definition velocities
vx = 0;
vy = 0;
vz = 0;

Vb = [vx;vy;vz];

%% Initial conditions angles
phi = 0*(pi/180);
theta = 0;
psi = 0*(pi/180);

angles_b = [phi;theta;psi];

%% Rotational Matrix
R_NC = Rot_zyx(angles_b);
R_total = zeros(3,3,length(t)+1);
R_total(:,:,1) = R_NC;

%% Angular velocities 
omega = [0; 0; 0];

H = [s;Vb];

%% System parameters
g = 9.8;
m = 1.25;

L = [g;m];

%% Inertial Value
I = [0.0232, 0, 0;...
     0, 0.0232, 0;...
     0 0 0.0468];

%% Desired reference z



%% Control values torques
T = [0.01*ones(1,length(t));
     0.0*ones(1,length(t));...
     0.00*ones(1,length(t));...
     0.00*ones(1,length(t))];

for k = 1:1:length(t)
    %% Error definition
    
    %% Control Law

    %% System evolution
    s(:,k+1) = system_linear_dynamics3D_DMD(s(:,k), T(:, k), A, B, t_s);
    
    h(1:3,k+1) = h(1:3,k)+s(1:3,k+1)*ts;
    
end


close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
%b) Dimenciones del Robot
   Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),s(4,1), s(5,1), s(6,1));hold on

%    plot3(H(1,1),H(2,1),H(3,11),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   


view(20,15);
for k = 1:20:length(t)-1
    drawnow
    delete(G2);
   
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),s(4,k), s(5,k), s(6,k));hold on
    
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    %plot3(obs(1,:),obs(2,:),obs(3,:),'x','Color',[0,171,217]/255,'linewidth',2);
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end


%% System pictures
fontsizeLegend = 20;

figure (2)

set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,h(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,h(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,h(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on

grid on;
legend({'$x$','$y$','$z$'},'Interpreter','latex','FontSize',fontsizeLegend,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure (3)
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,s(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,s(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,s(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on

grid on;
legend({'$hx_p$','$hy_p$','$hz_p$'},'Interpreter','latex','FontSize',fontsizeLegend,'Orientation','horizontal');
%legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$','$\mu_{l}$','$\mu_{m}$','$\mu_{n}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure (4)
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,s(4,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,s(5,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,s(6,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on


grid on;
legend({'$\phi$','$\theta$','$\psi$'},'Interpreter','latex','FontSize',fontsizeLegend,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
