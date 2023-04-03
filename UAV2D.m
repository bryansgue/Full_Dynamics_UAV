%% Vertical dynamics drone

%% clean variables 
clc, clear all, close all;

%% Time definition

t_s = 0.01;
t_final = 15;
t = (0:t_s:t_final);
%% Intitial conditions position

y = 0;
z = 0;
phi = 0*(pi/180);

h1 = [y;z;phi];
%% Initial condition definition velocities
y_p = 0;
z_p = 0;
phi_p = 0*(pi/180);

h2 = [y_p;z_p;phi_p];


h = [h1;h2];

%% System parameters
g = 9.8;
m = 1.25;



%% Inertial Value
I = [0.0232, 0, 0;...
     0, 0.0232, 0;...
     0 0 0.0468];
Ixx = 1.1232;
L = [g;m;Ixx];

zd = 1;
zd_p = 0;

yd = 0.5*cos(0.5*t);
yd_p = -0.5*sin(0.5*t);

% mul=0.1;
% [yd, zd, xd, psid, yd_p, zd_p, xd_p, psid_p] = Trayectorias(3,t,mul);


Kv = 10;
Kp= 10;


Kv2 = 10;
Kp2= 10;



%% Control values torques
F = [1.00*m*g*ones(1,length(t));...
     0.000*cos(t)];

for k = 1:1:length(t)
    %% Error definition
    
    
    
    %% Control Law
    ey   = yd(k) - h(1,k);
    ey_p = yd_p(k) - h(4,k);
    
    ez   = zd - h(2,k);
    ez_p = zd_p - h(5,k);
    
    phi_c(k) = (-1/g)*(Kv2*ey_p + Kp2*ey);
    
    if k<=1
        phi_cp = 0;
    else
        phi_cp = (phi_c(k)-phi_c(k-1))/t_s;
    end
    
    
    F(1,k) = m*(Kv*ez_p + Kp*ez + g);
    
    
    F(2,k) = Ixx*(Kv2*(phi_cp - h(6,k)) + Kp2*(phi_c(k)-h(3,k)) );
    
    
    
    
    %% System evolution
    h(:,k+1) = system_linear_dynamics2D(h(:,k), F(:, k), L, t_s);
  

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
    G2=Drone_Plot_3D(0,h(1,1),h(2,1),h(3,1), 0, 0);hold on

    plot3(h(1,1),h(2,1),h(3,11),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   


view(90,0);
for k = 1:20:length(t)-1
    drawnow
    delete(G2);
   
    G2=Drone_Plot_3D(0,h(1,k),h(2,k),h(3, k), 0, 0);hold on
    
    plot3(0*h(1,1:k),h(1,1:k),h(2,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    %plot3(obs(1,:),obs(2,:),obs(3,:),'x','Color',[0,171,217]/255,'linewidth',2);
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end


%% System pictures
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
%plot(t(1:length(he)),he(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
grid('minor')
grid on;
legend({'$\tilde{h_{z}}$','$\tilde{h_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,h(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,h(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,h(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on

grid on;
legend({'$y$','$z$','$\psi$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

grid on;
legend({'$\phi$','$\theta$','$\psi$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
%legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$','$\mu_{l}$','$\mu_{m}$','$\mu_{n}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,h(4,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,h(5,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,h(6,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on


grid on;
legend({'$v_x$','$v_y$','$v_z$','$v_{zd}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
