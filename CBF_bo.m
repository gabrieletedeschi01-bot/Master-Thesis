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
% gamma = 4;             % alpha(h) = gamma * h
gamma = 5;

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
    
    % if t>10 && t<15
    %     u_star(1)=u_star(1)+acc_max;
    % end

    

    % --- Followers ---
    for i = 2:N
        D = p(i) - p(i-1);   % inter-vehicle distance
        dv = v(i) - v(i-1);  % relative velocity


        % --- Nominal control ---
        Dvi_CP_dot = -kp * dv;
        Dvi_CP = -kp * (D - d_des);
        u_nom(i) =  u_star(i-1)+Dvi_CP_dot - kv*(dv - Dvi_CP) - (D - d_des) - rho(i);
        
        % if i==2 && t>10 && t<15
        %    u_star(1)=u_star(1)-acc_max;
        % end

        % if i==5 && t>10 && t<15
        %    u_nom(i)=u_nom(i)-acc_max;
        % end


        % if i==2 && (t>5 && t<30)
        %    u_nom(i)=u_nom(i)+10*sin(2*pi*0.1*t); %Ampiezza*sin(2*pi*frequenza*t);
        % end

        % if i==10 && (t>10 && t<15)  %veic=9
        %     u_nom(i) = u_nom(i)+acc_max;
        % end
       
        
        % --- CBF constraint ---
        h = -D  - tau_d*dv;

        h_dot= -dv -tau_d*(u_star(i)-u_star(i-1));

        

        % gamma_min=1;
        % gamma_max=0.1;
        % 
        % 
        % gamma_adapt = gamma_min + (gamma_max) * (h < 0);
        % alpha_h=gamma_adapt*h;

        alpha_h=gamma*h;

        % if h_dot+alpha_h<0
        %     fprintf('CBF condition violated at t=%.2f, vehicle %d\n', t, i);
        % end

        u_req(i) = u_star(i-1) - dv/tau_d + alpha_h/tau_d;

        if u_req(i) < -acc_max
           fprintf('QP infeasible at t=%.2f, vehicle %d\n', t, i);
        end


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
    dx(2)=u_star(1);

    % if t>10 && t<15
    %     dx(2)=u_nom(1)-acc_max;
    % end

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





%% Plots (same plotting structure as before, slightly cleaned)
function cmap = bluegreen(n)
    if nargin < 1
        n = 64;
    end
    cmap = [linspace(0,0,n)', linspace(0,1,n)', linspace(1,0,n)'];
end

colors = bluegreen(N);
% Distances (plotted as negative of dist to match your previous style)
figure(1); hold on; grid on;
title('Distance');
xlabel('Time [s]'); ylabel('Distance [m]');
for i = 1:N-1
    plot(t, -dist(:, i), 'Color', colors(i,:), 'LineWidth', 1.5);
end
legendStrings = arrayfun(@(i) sprintf('$p_{%d}-p_{%d}$', i, i-1), 1:N-1, 'UniformOutput', false);
legend(legendStrings, 'Interpreter', 'latex');

% Leader speed and speed differences
figure(2);
subplot(2,1,1);
plot(t, v(:,1), 'b', 'LineWidth', 1.5); grid on;
title('Speed of the leader'); xlabel('Time [s]'); ylabel('Speed [m/s]');

subplot(2,1,2); hold on; grid on;
for i = 2:N
    plot(t, v(:, i) - v(:, i-1), 'Color', colors(i,:), 'LineWidth', 1.2);
end
title('Difference of speed of the followers'); xlabel('Time [s]'); ylabel('Speed diff [m/s]');

% Control inputs comparison
figure; hold on; grid on;
for i = 2:N
    base_color = colors(i,:);
    req_color = min(base_color + 0.4, 1);
    plot(t, U_star(:, i), '-', 'Color', base_color, 'LineWidth', 1.5);
    plot(t, U_nom(:, i), '--', 'Color', base_color, 'LineWidth', 1.0);
    plot(t, U_req(:, i), ':', 'Color', req_color, 'LineWidth', 2.0);
end
yline(acc_max, '--k'); yline(-acc_max, '--k');
title('Control inputs comparison'); xlabel('Time [s]'); ylabel('u [m/s^2]');
% Legend (construct for followers)
leg = [];
for i = 2:N
    leg = [leg, {sprintf('$u_{star,%d}$', i-1), sprintf('$u_{nom,%d}$', i-1), sprintf('$u_{req,%d}$', i-1)}];
end
legend(leg, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'Best');

% CBF intervention magnitude
figure; hold on; grid on;
for i = 2:N
    plot(t, U_nom(:, i) - U_star(:, i),'Color', colors(i,:), 'LineWidth', 1.2);
end
title('CBF intervention magnitude'); xlabel('Time [s]'); ylabel('u_{nom} - u_{CBF}');
legend(arrayfun(@(i) sprintf('$\\Delta u_{%d}$', i-1), 2:N, 'UniformOutput', false), 'Interpreter', 'latex');

% Safety function h
figure; hold on; grid on;
for i = 1:N-1
    plot(t, h(:, i),'Color', colors(i,:), 'LineWidth', 1.2);
end
title('CBF safety function'); xlabel('Time [s]'); ylabel('$h = -\Delta p_i - \tau_d \Delta v_i$', 'Interpreter', 'latex');
legend(arrayfun(@(i) sprintf('h_{%d}', i), 1:N-1, 'UniformOutput', false));

%% End of script

