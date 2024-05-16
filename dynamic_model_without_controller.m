%% Dynamic Model of a land-based Mobile Robot

clear all;
clc;
close all;

%% Simulation Parameters
dt = 0.1; % Step-size
ts = 10; % Simulation Time
t = 0:dt:ts; % Time Span

%% Initial Conditions
eta0 = [0;
        0;
        0]; % Initial position and orientation of the vehicle

zeta0 = [0;
        0;
        0]; % Initial vector of input commands
eta(:,1)=eta0;
zeta(:,1)=zeta0;

%% Robot Parameters

m = 10; % Mass of the Mobile Robot
Iz = 0.1; % Inertia of the Mobile Robot
xcb=0; ycb=0; % Coordinates of C (Center of Mass)

%% State Propagation

for i=1:length(t)
    u=zeta(1,i); v=zeta(2,i); r=zeta(3,i);
    D=[ m,0,-ycb*m;
        0,m,xcb*m;
        -ycb*m,xcb*m,Iz+m*(xcb^2+ycb^2)]; % Inertia Matrix
    n_v=[-m*r*(v+xcb*r);
        m*r*(u-ycb*r);
        m*r*(xcb+ycb*v)]; % Other Vector
    tau(:,i)=[1;
        0.5;
        0]; %
    psi=eta(3,i);
    J_eta =  [cos(psi),-sin(psi),0;
        sin(psi),cos(psi),0;
        0,0,1]; % Jacobian Matrix
    zeta_dot(:,i) = inv(D)*(tau(:,i)-n_v); % Added semicolon to end the line
    zeta(:,i+1)=zeta(:,i)+dt*zeta_dot(:,i); % Velocity Update, changed zeta_dot(:,1) to zeta_dot(:,i)
    eta(:,i+1)=eta(:,i)+dt*(J_eta*zeta(:,i)+zeta_dot(:,i)*dt); % State Update, changed zeta(:,1) to zeta(:,i)
end

%% Animation
l=0.6; % Length of the Mobile Robot
w=0.4; % Width of the Mobile Robot
mr_co=[-l/2,l/2,l/2,-l/2,-l/2;
    -w/2,-w/2,w/2,w/2,-w/2]; % Mobile Robot Coordinates
figure
for i=1:length(t) % Animation starts here
    psi = eta(3,i);
    R_psi=[cos(psi),-sin(psi);
        sin(psi),cos(psi)]; % Rotation Matrix
    v_pos = R_psi*mr_co; % Changed ^ to *
    fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g') % Removed extra comma
    hold on, grid on
    l_lim = min(min(eta(1:2,:)));
    u_lim = max(max(eta(1:2,:)));
    axis([-0.5+l_lim, 0.5+u_lim, -0.5+l_lim, 0.5+u_lim]), axis square % Added commas
    plot(eta(1,1:i),eta(2,1:i),'b-');
    legend('MR','Path')
    set(gca,'fontsize',24)
    xlabel('x,[m]'); ylabel('y[m]');
    pause(0.1);
    hold off
end % Animation ends

%% Plotting Function
figure
plot(t,eta(1,1:length(t)),'r-',t,eta(2,1:length(t)),'b-.',t,eta(3,1:length(t)),'k--','LineWidth',2); % Changed 1:i to 1:length(t)
legend('x[m]','y[m]','\psi[rad]');
set(gca,'fontsize',24)
grid on
xlabel('t[s]')
ylabel('\eta_e,[units]');
