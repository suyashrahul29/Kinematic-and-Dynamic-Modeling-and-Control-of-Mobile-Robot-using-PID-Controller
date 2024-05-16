%% Clean the Workspace
clear all;
clc;
close all;


%% Simulation Parameters
dt = 0.1; % Step Size
ts = 20; % Simulation Time
t = 0:dt:ts; %time span


%% Vehicle Parameters
a = 0.05; % Radius of wheel
d = 0.1; % half of axle length


%% Initial Conditions
x0 = 0.5;
y0 = 0.5;
psi0 = pi/4;
eta0 = [x0;y0;psi0];

eta(:,1) = eta0;

omega = [0;0];


%% Control Parameters
k_p = 1.62; % 7.59



for i = 1:length(t)

    psi = eta(3,i);

    J_psi = [cos(psi), -sin(psi), 0;
             sin(psi), cos(psi), 0;
             0, 0, 1];

    W = [a/2 , a/2; 0, 0; -a/(2*d), a/(2*d)];

    %% Desired States
    eta_d(:,i) = [20*sin(t(i)); 20*cos(t(i)); -t(i)]; %[t(i); 2*t(i)^2; 4*t(i)]    %; %[2-2*cos(0.1*t(i)); 2*sin(0.1*t(i)); 0.1*t(i)];
    
    if i == 1
        eta_d_dot = [0;0;0];
    else
        eta_d_dot = (eta_d(:,i) - eta_d(:,i-1))/dt;
    end

    e(:,i) = eta_d(:,i) - eta(:,i);

    zeta(:,i) = W*omega;
    
    eta_dot(:,i) = J_psi * zeta(:,i);

    eta(:,i+1) = eta(:,i) + dt*eta_dot(:,i);
    
    zeta_d(:,i) = inv(J_psi) * (eta_d_dot + k_p * e(:,i));
    omega = pinv(W)*zeta_d(:,i);
    
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