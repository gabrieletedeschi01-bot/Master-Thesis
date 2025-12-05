%% platoon_cbf_continuous.m
% Continuous-time simulation of a platoon with nominal vs CBF-filtered control
% Integration via ODE45

clear; close all; clc;
%% Parameters
N = 10;                   % number of vehicles (1 leader + N-1 followers)
d_des = 3;              % desired spacing (m)
kp = 1; kv = 2;         % nominal controller gains
acc_max = 10;           % actuator bounds

tau_d = 1.8;             % headway-like parameter in h = D - tau_d * v_f
gamma = 5;             % alpha(h) = gamma * h

tspan=[0:10^(-1):30];               % simulation horizon
x0 = zeros(3*N,1);       % initial state [p1,v1,rho1,...,pN,vN,rhoN]

a=0.5;                   %macro function paramters
b=0.5;
l=1.5;
sum_p=0;
sum_v=0;



%% Initial conditions
v0 = 20; % initial speed [m/s]
p0 = 600; %initial position [m]
d_ini=3; %initial inter-vehic dist [m]
dv_ini=10;
for i=1:N
    x0(3*i-2) = p0 - (i-1)*d_ini;
    x0(3*i-1) = v0; % velocity
    x0(3*i)   = 0;           %control state

end





%% Leader input (function of time)
v_leader_fun = @(t)30;
%% Dynamics with CBF filter using quadprog
function [u_nom, u_star, u_req] = compute_control(t,p,v,rho,N,d_des,kp,kv,tau_d,gamma,acc_max,v_leader_fun)
    
    d_des=-d_des;
    
    u_nom = zeros(N,1);
    u_star = zeros(N,1);
    u_req = zeros(N,1);


    % --- Leader ---
    kl = 2;
    u_nom(1) = -kl*(v(1) - v_leader_fun(t)); % nominal leader control
    u_nom(1) = min(max(u_nom(1), -acc_max), acc_max); % clip to actuator limits
    u_star(1)=u_nom(1);

    % --- Followers ---
    for i = 2:N
        D = p(i) - p(i-1);   % inter-vehicle distance
        dv = v(i) - v(i-1);  % relative velocity

        % --- Nominal control ---
        Dvi_CP_dot = -kp * dv;
        Dvi_CP = -kp * (D - d_des);
        u_nom(i) =  u_star(i-1)+Dvi_CP_dot - kv*(dv - Dvi_CP) - (D - d_des) - rho(i);

        if i==10 && (t>10 && t<15)  %veic=9
            u_nom(i) = u_nom(i)+acc_max;
        end
   
        
        % --- CBF constraint ---
        h = -D  - tau_d*dv;

        % gamma_min=1;
        % gamma_max=0.1;
        % 
        % 
        % gamma_adapt = gamma_min + (gamma_max) * (h < 0);
        % alpha_h=gamma_adapt*h;

        alpha_h=gamma*h;

        u_req(i) = u_star(i-1) - dv/tau_d + alpha_h/tau_d;


        % --- QP formulation ---
        % Minimize (1/2)*(u - u_nom)^2 s.t. u <=u_req and actuator limits
        H = 1;       % 1x1 quadratic term
        f = -u_nom(i); % linear term
        A = 1;      % inequality: u <= u_req
        % b = u_req(i);
        b = max(u_req(i), -acc_max);
        lb = -acc_max;
        ub = acc_max;
        options = optimoptions('quadprog','Display','off');
        u_star(i)= quadprog(H,f,A,b,[],[],lb,ub,[],options);

    end
end


   

function [psi_p, psi_v]=macrofz(i,d_des,somma_p,somma_v,p_i,v_i)

    media_p=somma_p/(i);  %medie
    media_v=somma_v/(i);
    
    var_p=(p_i-media_p)^2/(i); %varianze
    var_v=(v_i-media_v)^2/(i);
    
    psi_p=sign(media_p+d_des)*sqrt(var_p); %macro function
    psi_v=sign(media_v)*sqrt(var_v);
end


   


                          
function dx = platoon_dyn(t,x,N,d_des,kp,kv,a,b,l,sum_p,sum_v,v_leader_fun,tau_d,gamma,acc_max)

    dx = zeros(3*N,1);
    % unpack states
    p = x(1:3:end);
    v = x(2:3:end);
    rho=x(3:3:end);


    [u_nom,u_star,u_req] = compute_control(t,p,v,rho, N, d_des, kp, kv, tau_d, gamma, acc_max, v_leader_fun);
    
    %-----Leader dyn-----%
    dx(1)=v(1);
    dx(2)=u_nom(1);

    if t>10 && t<15
        dx(2)=u_nom(1)+acc_max;
    end

    % followers
    for i=2:N
        D = p(i)-p(i-1);
        dv = v(i)-v(i-1);

        sum_p=sum_p+D;
        sum_v=sum_v+dv;
        [psi_p, psi_v]=macrofz(i-1,d_des,sum_p,sum_v,D,dv);


        dx(3*i-2) = v(i);
        dx(3*i-1) = u_star(i);
        dx(3*i)  = -l*x(3*i)+a*psi_p+b*psi_v;

        % if i==9 && (t>10 && t<15)  %veic=8
        %     dx(3*i-1) = u_nom(i)-acc_max;
        % end

    end
end

%% Simulation
opts = odeset('RelTol',1e-4,'AbsTol',1e-4,'MaxStep',10^-1);
odefun = @(t,x) platoon_dyn(t,x,N,d_des,kp,kv,a,b,l,sum_p,sum_v,v_leader_fun,tau_d,gamma,acc_max);
[t,X] = ode45(odefun,tspan,x0,opts);



% Extract states
p = X(:,1:3:end);
v = X(:,2:3:end);
rho=X(:,3:3:end);



%% Recompute controls at each time step
U_nom = zeros(length(t), N);
U_star = zeros(length(t), N);
U_req =  zeros(length(t), N);


for k = 1:length(t)
    [u_nom, u_star,u_req] = compute_control(t(k), p(k,:)', v(k,:)', rho(k,:)', N, d_des, kp, kv, tau_d, gamma, acc_max, v_leader_fun);
    U_nom(k,:) = u_nom;
    U_star(k,:) = u_star;
    U_req(k,:)= u_req;
end




dist=zeros(length(t),N);
h=zeros(length(t),N);
dv=zeros(length(t),N);

for i=2:N
    dist(:,i-1)=p(:,i)-p(:,i-1);
    dv(:,i-1) = v(:, i) - v(:, i-1);
    h(:,i-1)= -dist(:,i-1)  - tau_d*dv(:,i-1);
end

% for k = 1:length(t)
%     for i = 2:N
%         D = p(k,i) - p(k,i-1);
%         dist(k,i-1) = D;
%         h(k,i-1) = -D - tau_d*v(k,i);
%     end
% end





%% Plot Positions and Velocities
figure(1);
l = title("Distance");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = xlabel("Time [s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = ylabel("Distance [m]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
hold on


function cmap = bluegreen(n)
    if nargin < 1
        n = 64;
    end
    cmap = [linspace(0,0,n)', linspace(0,1,n)', linspace(1,0,n)'];
end

colors = bluegreen(N);

hold on
for i = 1:N-1
    plot(t, -dist(:,i), 'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', strcat('$p_{', string(i), '}-p_{',string(i-1), '}$'));
end

legend('Interpreter','Latex', 'FontSize', 16);
grid on




figure(2);
% Define the subplot for case i=2
subplot(2, 1, 1);
l = title("Speed of the leader");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = xlabel("Time [s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = ylabel("Speed [m/s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
hold on;

vel = v(:, 1);
plot(t, vel, 'b', 'LineWidth', 1.5);
grid on;
legend('$v_0$', 'FontSize', 16,'Interpreter','Latex','Location', 'Best')

subplot(2, 1, 2);
l= title("Difference of speed of the followers");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = xlabel("Time [s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = ylabel("Speed [m/s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
hold on;


for i = 2:N
    vel = v(:, i) - v(:, i-1);
    plot(t, vel, 'Color', colors(i,:), 'LineWidth', 1.5);
end
for i = 2:N
    s(i-1) = strcat(' $v_{', string(i-1), '}-v_{',string(i-2), '}$');
end
% for i=1:N
%     vel = v(:,i);
%     plot(t, vel, 'Color', colors(i,:), 'LineWidth', 1.5);
% end
% for i = 1:N
%     s(i) = strcat(' $v_{', string(i-1), '}$');
% end

l = legend(s);
set(l,'Interpreter','Latex','Orientation', 'vertical'); l.FontSize = 16;
grid on


%% Plot u_nom vs u_star (CBF)
% figure(3);
% hold on; grid on;
% for i = 2:N
%     plot(t, U_star(:,i), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
% end
% l = title("CBF-filtered control");
% set(l,'Interpreter','Latex'); l.FontSize = 16;
% xlabel("Time [s]", 'Interpreter','Latex', 'FontSize',16);
% ylabel("Control input $\mathrm{[m/s^2]}$", 'Interpreter','Latex', 'FontSize',16);
% s = strings(2*N,1);
% for i = 2:N
%     s(i-1) = strcat('$u_{star,', string(i-1), '}$');
% end
% legend(s, 'Interpreter','Latex','FontSize',14, 'Location','Best');
% 
% figure(4);
% hold on; grid on;
% for i = 2:N
%     plot(t, U_nom(:,i), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
% end
% l = title("Nominal control");
% set(l,'Interpreter','Latex'); l.FontSize = 16;
% xlabel("Time [s]", 'Interpreter','Latex', 'FontSize',16);
% ylabel("Control input $\mathrm{[m/s^2]}$", 'Interpreter','Latex', 'FontSize',16);
% s = strings(2*N,1);
% for i = 2:N
%     s(i-1) = strcat('$u_{nom,', string(i-1), '}$');
% end
% legend(s, 'Interpreter','Latex','FontSize',14, 'Location','Best');
% 
% 
% figure(5);
% hold on; grid on;
% for i = 2:N
%     plot(t, U_req(:,i), '--', 'Color', colors(i,:), 'LineWidth', 1.5);
% end
% l = title("Required control");
% set(l,'Interpreter','Latex'); l.FontSize = 16;
% xlabel("Time [s]", 'Interpreter','Latex', 'FontSize',16);
% ylabel("Control input $\mathrm{[m/s^2]}$", 'Interpreter','Latex', 'FontSize',16);
% s = strings(2*N,1);
% for i = 2:N
%     s(i-1) = strcat('$u_{req,', string(i-1), '}$');
% end
% legend(s, 'Interpreter','Latex','FontSize',14, 'Location','Best');

figure;
hold on; grid on;

vehicle_colors = lines(N-1);

for i = 2:N
    base_color = vehicle_colors(i-1,:);
    
    % Colori leggermente modificati per U_req (più chiari)
    req_color = min(base_color + 0.4, 1); % schiarisci
    
    % CBF-filtered control
    plot(t, U_star(:,i), '-', 'Color', base_color, 'LineWidth', 1.5);
    
    % Nominal control
    plot(t, U_nom(:,i), '--', 'Color', base_color, 'LineWidth', 1.5);
    
    % Required control (più chiaro, linea spessa e puntata)
    plot(t, U_req(:,i), ':', 'Color', req_color, 'LineWidth', 2.5);
end

q1 = yline(acc_max,  '--k', 'LineWidth', 2);
q2 = yline(-acc_max, '--k', 'LineWidth', 2);

% Titolo e assi
title("Control inputs comparison", 'Interpreter','Latex', 'FontSize',16);
xlabel("Time [s]", 'Interpreter','Latex', 'FontSize',16);
ylabel("Control input $\mathrm{[m/s^2]}$", 'Interpreter','Latex', 'FontSize',16);

% Legenda chiara
s = strings(3*(N+1),1);
for i = 2:N
    s(3*(i-2)+1) = strcat('$u_{star,', string(i-1), '}$');
    s(3*(i-2)+2) = strcat('$u_{nom,', string(i-1), '}$');
    s(3*(i-2)+3) = strcat('$u_{req,', string(i-1), '}$');
end

s(end-1) = "$+a_{\max}$";
s(end)   = "$-a_{\max}$";

legend(s, 'Interpreter','Latex','FontSize',12, 'Location','Best');


figure(6);
hold on; grid on;

for i = 2:N
    plot(t, U_nom(:,i) - U_star(:,i), 'LineWidth', 1.5, 'Color', colors(i,:));
end

xlabel('Time [s]'); ylabel('Delta control $u_{nom}-u_{CBF}$', 'Interpreter','Latex');
title('CBF intervention magnitude');
for i = 2:N
    s(i-1) = strcat('$\Delta u_{',num2str(i-1),'}$');
end
legend(s, 'Interpreter','Latex','FontSize',14, 'Location','Best');


%% plot safety function
figure(7); hold on; grid on;
for i=1:N-1
    plot(t, h(:,i), 'LineWidth', 1.5, 'Color', colors(i,:));
end
xlabel('Time [s]'); ylabel('$h = -\Delta p_i - \tau_d \Delta v_i$', 'Interpreter','Latex');
title('CBF safety function');
s = strings(N-1,1);
for i = 1:N
    s(i) = strcat('h_{', string(i), '}'); 
end
l = legend(s);
