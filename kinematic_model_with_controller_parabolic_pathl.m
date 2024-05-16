clear all; clc; close all;

% Import the Global Optimization Toolbox if it's not already imported
import globaloptim.*; % This imports all functions and classes from the Global Optimization Toolbox


%% Simulation Parameters
dt = 0.1; %Step Size
ts = 10; %Simulation Time
t = 0:dt:ts; %time span

%% Vehicle Paramters
a = 0.05; % Radius of wheel
d = 0.1; % half of axle length

%% Initial Conditions
x0 = 0.5;
y0 = 0.5;
psi0 = pi/4;

eta0 = [x0,y0,psi0];

eta(:,1) = eta0;

omega = [0;0];

% k_p = 1;

% Define saturation function (replace with your preferred method)
function y = saturate(x, min_val, max_val)
    y = max(min(x, max_val), min_val);
end

function v = Calculate(k_p)
dt = 0.1; %Step Size
ts = 10; %Simulation Time
t = 0:dt:ts; %time span

%% Vehicle Paramters
a = 0.05; % Radius of wheel
d = 0.1; % half of axle length

%% Initial Conditions
x0 = 0.5;
y0 = 0.5;
psi0 = pi/4;

eta0 = [x0,y0,psi0];

eta(:,1) = eta0;

omega = [0;0];

for i = 1:length(t) 

    psi = eta(3,i);

    J_psi = [cos(psi), -sin(psi), 0;
             sin(psi), cos(psi), 0;
             0, 0, 1];


    W = [a/2 , a/2; 0, 0; -a/(2*d), a/(2*d)];

    %% Desired States
    eta_d(:,i) = [t(i); 2*t(i)^2; atan(4*t(i))]; %[2-2*cos(0.1*t(i)); 2*sin(0.1*t(i)); 0.1*t(i)];  %
    
    if i == 1
        eta_d_dot = 0;
    else
        eta_d_dot = (eta_d(:,i) - eta_d(:,i-1))/dt;
    end

    e(:,i) = eta_d(:,i) - eta(:,i);

    zeta(:,i) = W*omega;
    
    eta_dot(:,i) = J_psi * zeta(:,i);

    eta(:,i+1) = eta(:,i) + dt*eta_dot(:,i);
    
    % var1=k_p .* e(:,i)
    % var2=k_i .* u_i * dt
    % var3=k_d .* u_d/ dt
    % var4=eta_d_dot
   
    zeta_d(:,i) = inv(J_psi) * (eta_d_dot + k_p * e(:,i));
    omega = pinv(W)*zeta_d(:,i);

    v(:,i) = e(:,i);
    %omega = [saturate(omega(1), -10, 10); saturate(omega(2), -10, 10)]
 
end
end

 

%% Animation
% Calculate(1);
% l = 0.4; %Length of mobile robot
% w = 0.2; %width of mobile Robot
% 
% %Coordinates of mobile robot
% mr_co = [-l/2,l/2,l/2,-l/2,-l/2;
%           -w/2, -w/2, w/2, w/2, -w/2];
% mr_co_head = [-l/2,l/6,l/6,-l/2,-l/2;
%                -w/2, -w/2, w/2, w/2, -w/2];
% figure
% for i = 1:length(t)
%     psi = eta(3,i);
%     R_psi = [cos(psi), -sin(psi);
%             sin(psi), cos(psi);];
%     v_pos = R_psi*mr_co;
%     %v_pos_head = R_psi*mr_co_head;
%     fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'b')
%     hold on, grid on
%     axis([eta(1,i)-5,eta(1,i)+5,eta(2,i)-5, eta(2,i)+5]) ,axis square
%     plot(eta(1, 1:i), eta(2,1:i), 'b-');
%     plot(eta_d(1,1:i), eta_d(2,1:i), 'k-');
%     %legend('MR','','Path','Desired Path')
%     set(gca, 'fontsize', 24)
%     xlabel('x,[m]'); ylabel('y,[m]');
%     pause(0.01);
%     hold off
% end
% 
% figure 
% plot(t,e)
% legend('x_e,[m]','y_e,[m]', '\psi_e,[rad]');
% set(gca,'fontsize',24)
% xlabel('t,[s]');
% ylabel('\eta_e,[units]');


function z = Optimize(k_p)
%% Simulation Parameters
    dt = 0.1; %Step Size
    ts = 10; %Simulation Time
    t = 0:dt:ts; %time span
    x = 0;
    for i = 1:length(t)
        e(:,i) = Calculate(k_p);
        x = x + e(:,i);
    end
    
    z = (x(1) + x(2) + x(3));
end

% Define lower and upper bounds for variables
lb = [0.01, 0.01];  % Lower bounds for variables
ub = [100, 100];    % Upper bounds for variables

% Set up the options for GA
options = optimoptions('ga', 'Display', 'iter');

% Run the genetic algorithm
[k_p, fval] = ga(Optimize, 2, [], [], [], [], lb, ub, [], options);
disp('Optimal Solution:');
disp(k_p);
disp('Minimum Value:');
disp(fval);


 % omega_1 = (u(1,i) + u(2,i) + u(3,i))/3
    % omega_2 = (u(1,i) + u(2,i) - u(3,i))/3

    % omega_1 = min(10, max(-10, omega_1));
    % omega_2 = min(10, max(-10, omega_2));
    % omega = [omega_1;omega_2];
    
    
    
    
    % k_zeta = 0.5;
     % zeta_f = k_zeta * u / dt; % zeta feedback
     % omega = pinv(W) * zeta_f(:,i);
     

    

    % % Updating Omega
    %  if i == 1
    %  else
    %     if (u(1,i) - u(1,i - 1)) ~= 0
    %         omega(1,1) = omega(1,1) + (u(1,i) - u(1,i - 1));
    %         omega(2,1) = omega(2,1) + (u(1,i) - u(1,i - 1));
    %     end
    % 
    %     if (u(2,i) - u(2,i - 1)) ~= 0
    %     end
    % 
    %     if (u(3,i) - u(3,i - 1)) ~= 0
    %         omega(1,1) = omega(1,1) - (u(3,i) - u(3,i - 1));
    %         omega(2,1) = omega(2,1) + (u(3,i) - u(3,i - 1));
    %     end
    % 
    %     if u(1,i) == 0 && u(2,i) == 0 && u(3,i) == 0
    %         omega = [0;0];
    %     end
    %  end
