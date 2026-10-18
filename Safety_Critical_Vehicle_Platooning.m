
%% platoon_cbf_continuous.m
% Continuous-time simulation of a platoon with nominal vs CBF-filtered control
% Integration via ODE45

clear; close all; clc;
%% Parameters
N = 11;                   % number of vehicles (1 leader + N-1 followers)
d_des = 3;              % desired spacing (m)
kp = 1; kv = 2;         % nominal controller gains
acc_max = 15;           % actuator bounds

tau_d = 1.8;             % headway-like parameter in h = D - tau_d * v_f
% gamma = 0.5;             % alpha(h) = gamma * h
gamma=45;

tspan=[0:10^(-1):30];               % simulation horizon
x0 = zeros(3*N,1);       % initial state [p1,v1,rho1,...,pN,vN,rhoN]

a=0.5;                   %macro function paramters
b=0.5;

a1=0.5;
b1=0.5;
l=1.5;
sum_p=0;
sum_v=0;
sum_u=0;

global qp_times
qp_times = [];


%% Initial conditions
v0 = 0; % initial speed [m/s]
p0 = 600; %initial position [m]
d_ini=3; %initial inter-vehic dist [m]
dv_ini=2;
% for i=1:N
%     x0(3*i-2) = p0 - (i-1)*d_ini;
%     x0(3*i-1) = v0; % velocity
%     x0(3*i)   = 0;           %control state
% 
% end

rng(1);

for i=1:N
    x0(3*i-2) = p0 - (i-1)*d_ini + 0.5*randn;

    x0(3*i-1) = 10+2*sin(i);

    x0(3*i) = 0;
end


%% Leader input (function of time)
%v_leader_fun = @(t)10;
v_leader_fun = @(t) (t<5)*15 + (t>=5)*5;
%% Dynamics with CBF filter using quadprog
function [u_nom, u_star, u_req] = compute_control(t,p,v,rho,N,d_des,kp,kv,tau_d,gamma,acc_max,v_leader_fun,a1,b1)
    
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
        u_nom(i) = u_star(i-1) + Dvi_CP_dot - kv*(dv - Dvi_CP) - (D - d_des) - rho(i);


        % if i==2 && (t>5 && t<30)
        %    u_star(1)=u_star(1)+10*sin(2*pi*0.1*t); %Ampiezza*sin(2*pi*frequenza*t);
        % end
        % 
        % if i==11 && (t>10 && t<15)  %veic=5
        %     u_star(i-1)= u_star(i-1)-8;
        % end

       

        % --- CBF constraint ---
        if i==2
            D_1 = p(i-1);      % D_i = p_i - p_{i-1}
            dv_1 = v(i-1) - v_leader_fun(t);     % dv_i = v_i - v_{i-1}
            du_1= u_star(i-1);
    
            % medie
            media_D1 = mean(D_1);
            media_dv1 = mean(dv_1);
            media_u1=mean(du_1);
    
            % varianze
            var_D1 = mean((D_1 - media_D1).^2);
            var_dv1 = mean((dv_1 - media_dv1).^2);
    
            % macrofunzioni
            psi_p = sign(media_D1 - d_des) * sqrt(var_D1);
            psi_v = sign(media_dv1) * sqrt(var_dv1);
    
    
    
            dmedia_D=media_dv1;
            dmedia_v=media_u1;
    
            tol = 1e-3;
            if var_D1 < tol
                dpsi_p = 0;
            else
                dvar_p = 2 .* (D_1 - media_D1) .* (dv_1 - dmedia_D);
                dpsi_p=sign(media_D1+d_des).*(1/2).*(1/sqrt(var_D1)).*dvar_p;
            end
        
            if var_dv1 < tol
                dpsi_v = 0;
            else
                dvar_v = 2 .* (dv_1 - media_dv1) .* (du_1 - dmedia_v);
                dpsi_v=sign(media_dv1).*(1/2).*(1/sqrt(var_dv1)).*dvar_v;
            end
       
            h = -D - tau_d*dv - ((v(i) - v(i-1))^2)/(2*acc_max) + a1*psi_p + b1*psi_v;
         
            alpha_h=gamma*h;

            dh_dv = -tau_d - (v(i)-v(i-1))/acc_max;
            u_req(i) =u_star(i-1) -dv/-dh_dv + (a1*dpsi_p)/-dh_dv + (b1*dpsi_v)/-dh_dv + alpha_h/-dh_dv;
        else
           
            D_vect = p(2:i-1) - p(1:i-2);      % D_i = p_i - p_{i-1}
            dv_vect = v(2:i-1) - v(1:i-2);     % dv_i = v_i - v_{i-1}
            du_vect= u_star(i-1)- u_star(i-2);
    
            % medie
            media_D = mean(D_vect);
            media_dv = mean(dv_vect);
            media_u=mean(du_vect);
    
            % varianze
            var_D = mean((D_vect - media_D).^2);
            var_dv = mean((dv_vect - media_dv).^2);
    
            % macrofunzioni
            psi_p = sign(media_D - d_des) * sqrt(var_D);
            psi_v = sign(media_dv) * sqrt(var_dv);
    
    
    
            dmedia_D=media_dv;
            dmedia_v=media_u;
    
            tol = 1e-3;
            if var_D < tol
                dpsi_p = 0;
            else
                dvar_p = 2 .* (D_vect - media_D) .* (dv_vect - dmedia_D);
                dpsi_p=sign(media_D+d_des).*(1/2).*(1/sqrt(var_D)).*dvar_p;
            end
        
            if var_D < tol
                dpsi_v = 0;
            else
                dvar_v = 2 .* (dv_vect - media_dv) .* (du_vect - dmedia_v);
                dpsi_v=sign(media_dv).*(1/2).*(1/sqrt(var_dv)).*dvar_v;
            end
       
            h = -D - tau_d*dv - ((v(i) - v(i-1))^2)/(2*acc_max) + a1*psi_p +b1*psi_v;
    
            alpha_h=gamma*h;

            dh_dv = -tau_d - (v(i)-v(i-1))/acc_max;
            u_req(i) =u_star(i-1) -dv/-dh_dv + (a1*dpsi_p(end))/-dh_dv + (b1*dpsi_v(end))/-dh_dv + alpha_h/-dh_dv;

        end 
      

        %  --- QP formulation ---
        % Minimize (1/2)*(u - u_nom)^2 s.t. u <=u_req and actuator limits
        H = 1;       % 1x1 quadratic term
        f = -u_nom(i); % linear term
        A = 1;      % inequality: u <= u_req
        % b = u_req(i);
        b = max(u_req(i), -acc_max);
        lb = -acc_max;
        ub = acc_max;
        % options = optimoptions('quadprog','Display','off');
        % u_star(i) = quadprog(H,f,A,b,[],[],lb,ub,[],options);
        options = optimoptions('quadprog','Display','off');

        global qp_times
        
        tic;
        u_star(i) = quadprog(H,f,A,b,[],[],lb,ub,[],options);
        qp_times(end+1) = toc;


    end
end


   

function [psi_p, psi_v]=macrofz(i,d_des,somma_p,somma_v,p_i,v_i)
    tol = 1e-8;

    media_p=somma_p/(i);  %medie
    media_v=somma_v/(i);
    
    var_p=(p_i-media_p)^2/(i); %varianze
    var_v=(v_i-media_v)^2/(i);
    
    psi_p=sign(media_p+d_des)*sqrt(var_p); %macro function
    psi_v=sign(media_v)*sqrt(var_v);

 

end


                          
function dx = platoon_dyn(t,x,N,d_des,kp,kv,a,b,l,sum_p,sum_v,sum_u,v_leader_fun,tau_d,gamma,acc_max,a1,b1)

    dx = zeros(3*N,1);
    % unpack states
    p = x(1:3:end);
    v = x(2:3:end);
    rho=x(3:3:end);

  


    [u_nom,u_star,u_req] = compute_control(t,p,v,rho, N, d_des, kp, kv, tau_d, gamma, acc_max, v_leader_fun,a1,b1);
    
    %-----Leader dyn-----%
    dx(1)=v(1);
    dx(2)=u_star(1);

    if t>5 && t<30
        dx(2)=u_star(1)+5*sin(2*pi*0.1*t); %Ampiezza*sin(2*pi*frequenza*t);;
    end

    % if t>10 && t<15
    %     dx(2)=u_star(1)-8;
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

        if i==10 && (t>10 && t<15)  %veic=9
            dx(3*i-1)= u_star(i)-10;
        end
    end
end

%% Simulation
opts = odeset('RelTol',1e-4,'AbsTol',1e-4,'MaxStep',10^-1);
odefun = @(t,x) platoon_dyn(t,x,N,d_des,kp,kv,a,b,l,sum_p,sum_v,sum_u,v_leader_fun,tau_d,gamma,acc_max,a1,b1);
[t,X] = ode45(odefun,tspan,x0,opts);

global qp_times

avg_qp_time = mean(qp_times);
max_qp_time = max(qp_times);

fprintf('Average QP solve time: %.6f s\n', avg_qp_time);
fprintf('Maximum QP solve time: %.6f s\n', max_qp_time);
fprintf('Median QP solve time: %.6f s\n', median(qp_times));



% Extract states
p = X(:,1:3:end);
v = X(:,2:3:end);
rho=X(:,3:3:end);



%% Recompute controls at each time step
U_nom = zeros(length(t), N);
U_star = zeros(length(t), N);
U_req =  zeros(length(t), N);




for k = 1:length(t)
    [u_nom, u_star,u_req] = compute_control(t(k), p(k,:)', v(k,:)', rho(k,:)', N, d_des, kp, kv, tau_d, gamma, acc_max, v_leader_fun,a1,b1);
    U_nom(k,:) = u_nom;
    U_star(k,:) = u_star;
    U_req(k,:)= u_req;
end




dist=zeros(length(t),N);
h=zeros(length(t),N);

% for i=2:N
%     dist(:,i-1)=p(:,i)-p(:,i-1);
%     [psi_p, psi_v] = macrofz(i-1, d_des, sum_p, sum_v, D, dv);
%     dv(:,i-1) = v(:, i) - v(:, i-1);
%     h(:,i-1)= -dist(:,i-1) - tau_d*dv(:,i-1);
% end
% a1=1;
% b1=1;

for k = 1:length(t)
    for i = 2:N
        D = p(k,i) - p(k,i-1);
        dist(k,i-1) = D;
        dv(k,i-1) = v(k, i) - v(k, i-1);
        [psi_p, psi_v] = macrofz(i-1, d_des, sum_p, sum_v, dist(k,i-1), dv(k,i-1));
        if i==2
            D_1 = p(i-1);      % D_i = p_i - p_{i-1}
            dv_1 = v(i-1) - v_leader_fun(t);     % dv_i = v_i - v_{i-1}
            du_1= u_star(i-1);
    
            % medie
            media_D1 = mean(D_1);
            media_dv1 = mean(dv_1);
            media_u1=mean(du_1);
    
            % varianze
            var_D1 = mean((D_1 - media_D1).^2);
            var_dv1 = mean((dv_1 - media_dv1).^2);
    
            % macrofunzioni
            psi_p = sign(media_D1 - d_des) * sqrt(var_D1);
            psi_v = sign(media_dv1) * sqrt(var_dv1);
            h(k,i-1)= -D - tau_d*dv(k,i-1)-  ((v(k,i) - v(k,i-1))^2)/(2*acc_max) + a1*psi_p + b1*psi_v ;
        else
             h(k,i-1) = -D - tau_d*dv(k,i-1) - ((v(k,i) - v(k,i-1))^2)/(2*acc_max)+a1*psi_p +b1*psi_v;
        end
    end
end





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
l= title("Distance");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = xlabel("Time [s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = ylabel("Distance [m]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
hold on;
for i = 1:N-1
    plot(t, -dist(:, i), 'Color', colors(i,:), 'LineWidth', 1.5,...
        'DisplayName', strcat('$p_{', string(i), '}-p_{',string(i-1), '}$'));
end
yline(0, '--k', 'LineWidth', 2, 'HandleVisibility','off');
text(t(22), 0, 'Collision', ...
    'Interpreter','latex', ...
    'FontSize', 12, ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','bottom', ...
    'Color','k');



legend('Interpreter','Latex', 'FontSize', 12);
grid on
%% Leader speed and speed differences
figure(2);
subplot(2,1,1);
plot(t, v(:,1), 'b', 'LineWidth', 1.5); grid on;
l = title("Speed of the leader");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = xlabel("Time [s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = ylabel("Speed [m/s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;

subplot(2,1,2);  hold on; grid on;
for i = 2:N
    plot(t, v(:, i) - v(:, i-1), 'Color', colors(i,:), 'LineWidth', 1.2);
end
l= title("Difference of speed");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = xlabel("Time [s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
l = ylabel("Speed [m/s]");
set(l,'Interpreter','Latex'); l.FontSize = 16;
hold on;
grid on;

legendStrings = arrayfun(@(i) sprintf('$v_{%d}-v_{%d}$', i, i-1), 1:N-1, 'UniformOutput', false);
legend(legendStrings, 'Interpreter', 'latex');

%% Control inputs comparison
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

%% Safety function h
figure; hold on; grid on;
for i = 1:N-1
    plot(t, h(:, i),'Color', colors(i,:), 'LineWidth', 1.2);
end
yline(0, '--k', 'LineWidth', 2, 'HandleVisibility','off');

text(t(50), 0, 'Safe set', ...
    'Interpreter','latex', ...
    'FontSize', 12, ...
    'HorizontalAlignment','right', ...
    'VerticalAlignment','bottom', ...
    'Color','k');
title('CBF safety function', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$h_i = -\Delta p_i - \tau_d \Delta v_i - \frac{(\Delta v_i)^2}{2 a_{\max}}+ a\psi_{\Delta p}^{i-1}+b \psi_{\Delta v}^{i-1}$', ...
       'Interpreter', 'latex');
legend(arrayfun(@(i) sprintf('$h_{%d}$', i), 1:N-1, ...
       'UniformOutput', false), ...
       'Interpreter', 'latex');


%% Compute and plot CBF condition h_dot + alpha(h)

% h_dot = zeros(length(t), N-1);
% cond = zeros(length(t), N-1);
% 
% for i = 2:N
%     % h = -D - tau_d * dv  → D = p_i - p_(i-1) , dv = v_i - v_(i-1)
%     D  = p(:, i) - p(:, i-1);
%     dv = v(:, i) - v(:, i-1);
% 
%     % h(i)
%     h_i = -D - tau_d * dv;
% 
%     % u_star(i) and u_star(i-1)
%     ui     = U_star(:, i);
%     uimin1 = U_star(:, i-1);
% 
%     % h_dot(i) = -dv - tau_d*(u_i - u_{i-1})
%     h_dot(:, i-1) = -dv - tau_d * (ui - uimin1);
% 
%     % alpha = gamma * h
%     alpha_h = gamma * h_i;
% 
%     % CBF condition = h_dot + alpha(h)
%     cond(:, i-1) = h_dot(:, i-1) + alpha_h;
% end
% 
% % Plot
% figure; hold on; grid on;
% title('CBF Condition $\dot{h} + \gamma h$', 'Interpreter','latex');
% xlabel('Time [s]');
% ylabel('$\dot{h} + \gamma h$', 'Interpreter','latex');
% 
% for i = 1:N-1
%     plot(t, cond(:, i), 'LineWidth', 1.4);
% end
% yline(0,'--k','LineWidth',1.5);
% 
% legend(arrayfun(@(i) sprintf('Vehicle %d', i+1), 1:N-1,'UniformOutput',false));


%% End of script

