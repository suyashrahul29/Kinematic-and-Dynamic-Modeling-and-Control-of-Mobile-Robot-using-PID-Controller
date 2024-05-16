clear all;
clc;
close all;

%% Simulation Parameters
dt = 0.1; % Step-size
ts = 20; % Simulation Time
t = 0:dt:ts; % Time Span

%% Vehicle Parameters
a = 0.05; % Radius of wheel
d = 0.1; % half of axle length
W = [a/2 , a/2; 0, 0; -a/(2*d), a/(2*d)];

%% Initial Conditions
x0 = 0.5;
y0 = 0.5;
psi0 = pi/4;
eta0 = [x0;y0;psi0]; % Initial position and orientation of the vehicle
eta(:,1) = eta0;

omega = [0;0];

zeta0 = [0; 0; 0]; % Initial vector of input commands
zeta(:,1) = zeta0;

%% Control Parameters
k_p = 1.00001;
q_p = 0.00003;
q_d = 0.00009;
c_p = q_p*[1,0,0;0,1,0;0,0,1];
c_d = q_d*[1,0,0;0,1,0;0,0,1];

%% Robot Parameters
m = 10; % Mass of the Mobile Robot
Iz = 0.1; % Inertia of the Mobile Robot
xcb = 0; ycb = 0; % Coordinates of C (Center of Mass) wrt Body Frame

%% State Propagation
for i = 1:length(t)
    %% Desired States
    eta_d(:,i) =  [sin(t(i)); cos(t(i)); -t(i)]; % [t(i); 2*t(i)^2; atan(4*t(i))]; %
    
    if i == 1
        eta_d_dot(:,i) = [0; 0; 0];
    else
        eta_d_dot(:,i) = (eta_d(:,i) - eta_d(:,i-1)) / dt;
    end
    
    if i == 1
        eta_d_dot_dot(:,i) = [0; 0; 0];
    else
        eta_d_dot_dot(:,i) = (eta_d_dot(:,i) - eta_d_dot(:,i-1)) / dt;
    end
    
    psi = eta(3,i);
    J_psi =  [cos(psi),-sin(psi),0;
              sin(psi),cos(psi),0;
              0,0,1]; % Jacobian Matrix
          
    e(:,i) = eta_d(:,i) - eta(:,i);
    
    zeta(:,i) = W * omega;
    if i == 1
        zeta_dot(:,i) = [0; 0; 0];
    else
        zeta_dot(:,i) = (zeta(:,i) - zeta(:,i-1)) / dt;
    end
    
    eta_dot(:,i) = J_psi * zeta(:,i);
    
    zeta_d(:,i) = pinv(J_psi) * (eta_d_dot(:,i) + k_p * e(:,i));
    if i == 1
        zeta_d_dot(:,i) = [0; 0; 0];
    else
        zeta_d_dot(:,i) = (zeta_d(:,i) - zeta_d(:,i-1)) / dt;
    end
    
    u = zeta_d(1,i); v = zeta_d(2,i); r = zeta_d(3,i);
    D = [ m, 0, -ycb*m;
          0, m, xcb*m;
          -ycb*m, xcb*m, Iz+m*(xcb^2+ycb^2)]; % Inertia Matrix
    n_v = [-m*r*(v+xcb*r);
           m*r*(u-ycb*r);
           m*r*(xcb+ycb*v)]; % Other Vector
    tau(:,i) = D * zeta_d_dot(:,i) + n_v; % Input Vector
    
    e2(:,i) = eta_d_dot(:,i) - eta_dot(:,i);
    tau_d(:,i) = (J_psi') * (c_p * e(:,i) + c_d * e2(:,i));
    
    %zeta_d(:,i) = pinv(D) * (tau_d(:,i) - n_v);
    omega = pinv(W) * zeta_d(:,i);
    
    psi_d = eta_d(3,i);
    J_psi_d =  [cos(psi_d),-sin(psi_d),0;
                sin(psi_d),cos(psi_d),0;
                0,0,1]; % Jacobian Matrix
    
    % if i == 1
    %     J_psi_d_dot = [0;0;0];
    % else
    %     J_psi_d_dot = (J_psi_d(:,i) - J_psi_d(:,i-1)) / dt;
    % end
    
    zeta_dot(:,i) = D \ (tau(:,i) - n_v);
    zeta(:,i+1) = zeta(:,i) + dt * zeta_dot(:,i); % Velocity Update by Euler Integration
    eta(:,i+1) = eta(:,i) + dt * (J_psi * zeta(:,i) + zeta_dot(:,i) * dt); % State Update by Euler Integration
end

%% Animation Parameters
 
 l = 0.4; %Length of mobile robot
 w = 0.2; %width of mobile Robot
 
 %% Coordinates of Mobile Robot
 mr_co = [-l/2,l/2,l/2,-l/2,-l/2;
           -w/2, -w/2, w/2, w/2, -w/2];
 mr_co_head = [-l/2,l/6,l/6,-l/2,-l/2;
                -w/2, -w/2, w/2, w/2, -w/2];
 
 %% GRAPH: Mobile Robot chasing the Desired Path
 figure
 for i = 1:length(t)
     psi = eta(3,i);
     R_psi = [cos(psi), -sin(psi);
             sin(psi), cos(psi);];
     v_pos = R_psi*mr_co;
     v_pos_head = R_psi*mr_co_head;
     fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'b',v_pos_head(1,:)+eta(1,i),v_pos_head(2,:)+eta(2,i),'g')
     hold on, grid on
     axis([eta(1,i)-5,eta(1,i)+5,eta(2,i)-5, eta(2,i)+5]) ,axis square
     plot(eta(1, 1:i), eta(2,1:i), 'b-');
     plot(eta_d(1,1:i), eta_d(2,1:i), 'k-');
     %legend('MR','','Path','Desired Path')
     set(gca, 'fontsize', 24)
     xlabel('x,[m]'); ylabel('y,[m]');
     pause(0.0001);
     hold off
 end

 
 %% GRAPH: Mobile Robot chasing the Desired Path (Zoomed-out view)
 figure
 for i = 1:length(t)
     psi = eta(3,i);
     R_psi = [cos(psi), -sin(psi);
             sin(psi), cos(psi);];
     v_pos = R_psi*mr_co;
     v_pos_head = R_psi*mr_co_head;
     fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'b',v_pos_head(1,:)+eta(1,i),v_pos_head(2,:)+eta(2,i),'g')
     hold on, grid on
     
     % Calculate min and max coordinates
     min_x = min(eta(1,:)) - 2;
     max_x = max(eta(1,:)) + 2;
     min_y = min(eta(2,:)) - 2;
     max_y = max(eta(2,:)) + 2;
     
     axis([min_x, max_x, min_y, max_y]); % Set axis limits
     axis square
     
     plot(eta(1, 1:i), eta(2,1:i), 'b-');
     plot(eta_d(1,1:i), eta_d(2,1:i), 'k-');
     %legend('MR','','Path','Desired Path')
     set(gca, 'fontsize', 24)
     xlabel('x,[m]'); ylabel('y,[m]');
     pause(0.0001);
     hold off
 end


 %% GRAPH: Time vs Error
 figure 
 plot(t,e)
 legend('x_e,[m]','y_e,[m]', '\psi_e,[rad]');
 set(gca,'fontsize',24)
 xlabel('t,[s]');
 ylabel('\eta_e,[units]');
