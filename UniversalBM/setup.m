clear
close all
addpath(genpath(pwd))

%% Switch winding that you develop
load_system('BearinglessMotorSimulation') % load Simulink model
load_system("Plant")
load_system("Controller")
% The input of this function should be 'Separate', 'MP', 'Bridge',
% 'Parallel', or 'MCI'
winding_configuration = "MCI";

winding_conf_dic = dictionary("Separate", 1, ...
                              "MP", 2, ...
                              "MP2", 3, ...
                              "Bridge", 4, ...
                              "Parallel", 5, ...
                              "MCI", 6);
winding_conf_num = winding_conf_dic(winding_configuration);

Tend = 0.03; % Simulation stop time
Tsim = 1e-5; % Simulation sampling time
V_DC = 160; % DC bus (V)

torque_cmd = 0.2; % Torque reference (Nm)
torque_start = (1/6)*Tend; % Torque start time (s)
torque_end = (4/6)*Tend; % Torque end time (s)

fx_cmd = 9; % fx force reference (N)
fx_start = 0; % x-axis force start time (s)
fx_end = (2/6)*Tend; % x-axis force end time (s)
fy_cmd = 9; % fy force reference (N)
fy_start = (3/6)*Tend; % y-axis force start time (s)
fy_end = (5/6)*Tend; % y-axis force end time (s)

id_cmd = 0; % id current reference (A)
id_start = 0; % d-axis current start time (s)
id_end = 0.025; % d-axis current end time (s)

rotor_unbalance_weight = 1; % (kg)
rotor_unbalance_pos = 0; % epsilon (m)

speed_cmd = 7500; % Speed command (r/min)

%% Plant parameters
p = 4; % Number of torque pole-pair
ps = 5; % Number of suspension pole-pair

m = 6;  % Number of phase
alpha_t = wrapToPi(2*pi/m*p);
alpha_s = wrapToPi(2*pi/m*ps);

R = 0.3; % Phase resistance (Ohm)
Rmat = R*eye(6, 6);

% Leakage inductance 1%
Lt = 300e-6; % Self-inductance of torque system without leakage
Ls = 450e-6; % Self-inductance of suspension system without leakage
Llkg = 0.01*Lt;

% calculate Generalized Clarke Transform
Cm = 2/m;
Cr = Cm*[cos(0*alpha_t) cos(1*alpha_t) cos(2*alpha_t) cos(3*alpha_t) cos(4*alpha_t) cos(5*alpha_t);
         sin(0*alpha_t) sin(1*alpha_t) sin(2*alpha_t) sin(3*alpha_t) sin(4*alpha_t) sin(5*alpha_t);
         cos(0*alpha_s) cos(1*alpha_s) cos(2*alpha_s) cos(3*alpha_s) cos(4*alpha_s) cos(5*alpha_s);
         sin(0*alpha_s) sin(1*alpha_s) sin(2*alpha_s) sin(3*alpha_s) sin(4*alpha_s) sin(5*alpha_s);
         1 1 1 1 1 1;
         1 -1 1 -1 1 -1;];

% Calculate Lmat for MP configuration
Lmat = 2/m*[Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(4*alpha_t)+Ls*cos(4*alpha_s) Lt*cos(5*alpha_t)+Ls*cos(5*alpha_s);
            Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(4*alpha_t)+Ls*cos(4*alpha_s);
            Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s);
            Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s);
            Lt*cos(4*alpha_t)+Ls*cos(4*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s);
            Lt*cos(5*alpha_t)+Ls*cos(5*alpha_s) Lt*cos(4*alpha_t)+Ls*cos(4*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls;] ...
        + Llkg*eye(6);

% Need to update Lt and Ls to include Llkg 
L_alpha_beta = Cr*Lmat/Cr;
Lt = L_alpha_beta(1,1);
Ls = L_alpha_beta(3,3);

epsilon = 1e-2;
if (abs(alpha_t - pi/3) < epsilon && abs(alpha_s - 2*pi/3) < epsilon) % case 1
    map_case = [-4 1 -6 3 -2 5];
elseif (abs(alpha_t - 2*pi/3) < epsilon && abs(alpha_s - pi/3) < epsilon) % case 2
    map_case = [4 1 2 5 6 3];
elseif (abs(alpha_t + pi/3) < epsilon && abs(alpha_s + 2*pi/3) < epsilon) % case 3
    map_case = [-4 1 -2 5 -6 3];
elseif (abs(alpha_t + 2*pi/3) < epsilon && abs(alpha_s + pi/3) < epsilon) % case 4
    map_case = [4 1 6 3 2 5];
end

%% Machine constants
Ke = 0.003;  % back-emf constant (Vpk/(mech rad/s))
% Ke = 0;  % back-emf constant (Vpk/(mech rad/s))
Kt = 0.02; % Torque constant (Nm/Apk)
Kf = 1.8; % Force constant (N/Apk)

tau_hat = 2/m*Kt;

Fx_hat = 2/m*Kf;
if ps > p % ps = p + 1
    Kis = 1;
    conf = 'p+1';
elseif ps < p % ps = p - 1
    Kis = -1;
    conf = 'p-1';
end
Fy_hat = -Kis * 2/m*Kf;

switch(winding_configuration)

    case "MP"
        T_ph_term_v = eye(m,m);
        T_mp_term_v = eye(m,m);
        T_term_mp_v = T_mp_term_v^(-1);
        T_mp_term_i = eye(m,m);
        T_term_mp_i = T_mp_term_i^(-1);
        T_m_mp = eye(m,m);
        T_mp_m = T_m_mp^(-1);
    
    case "MP2"
        T_mp_term_v = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1];
        T_term_mp_v = T_mp_term_v^(-1);
        T_mp_term_i = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1];
        T_term_mp_i = T_mp_term_i^(-1);
        T_m_mp = zeros(m,m);
        for row = 1:m
            T_m_mp(row, abs(map_case(row))) = sign(map_case(row));
        end
        T_mp_m = T_m_mp^(-1);
    
    case "Parallel"
        T_mp_term_v = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1; -1 1 0 0 0 0; 0 0 -1 1 0 0; 0 0 0 0 -1 1];
        T_term_mp_v = T_mp_term_v^(-1);
        T_mp_term_i = [1 1 0 0 0 0; 0 0 1 1 0 0; 0 0 0 0 1 1; -1 0 0 0 0 0; 0 0 -1 0 0 0; 0 0 0 0 -1 0];
        T_term_mp_i = T_mp_term_i^(-1);
        T_m_mp = zeros(m,m);
        for row = 1:m
            T_m_mp(row, abs(map_case(row))) = sign(map_case(row));
        end
        T_mp_m = T_m_mp^(-1);

    case "MCI"
        T_mp_term_v = [1 1 0 0 0 0; 0 0 1 1 0 0; 0 0 0 0 1 1; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1];
        T_term_mp_v = T_mp_term_v^(-1);
        T_mp_term_i = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0; -1 1 0 0 0 0; 0 0 -1 1 0 0; 0 0 0 0 -1 1];
        T_term_mp_i = T_mp_term_i^(-1);
        T_m_mp = zeros(m,m);
        for row = 1:m
            T_m_mp(row, abs(map_case(row))) = sign(map_case(row));
        end
        T_mp_m = T_m_mp^(-1);
    
    case "Bridge"
        T_mp_term_v = [1 0 0 1 0 0 0 0 0 0 0 0; ...
                        0 0 0 0 1 0 0 1 0 0 0 0; ...
                        0 0 0 0 0 0 0 0 1 0 0 1; ...
                        0 0 -1 1 0 0 0 0 0 0 0 0; ...
                        0 0 0 0 0 0 -1 1 0 0 0 0; ...
                        0 0 0 0 0 0 0 0 0 0 -1 1];
        T_term_mp_v = 0.5*[1 0 0 -1 0 0; ...
                            1 0 0 1 0 0; ...
                            1 0 0 -1 0 0; ...
                            1 0 0 1 0 0; ...
                            0 1 0 0 -1 0; ...
                            0 1 0 0 1 0; ...
                            0 1 0 0 -1 0; ...
                            0 1 0 0 1 0; ...
                            0 0 1 0 0 -1; ...
                            0 0 1 0 0 1; ...
                            0 0 1 0 0 -1; ...
                            0 0 1 0 0 1];
        T_mp_term_i = 2*1*[1 1 0 0 0 0 0 0 0 0 0 0; ...
                        0 0 0 0 1 1 0 0 0 0 0 0; ...
                        0 0 0 0 0 0 0 0 1 1 0 0; ...
                        -1 0 0 1 0 0 0 0 0 0 0 0; ...
                        0 0 0 0 -1 0 0 1 0 0 0 0; ...
                        0 0 0 0 0 0 0 0 -1 0 0 1];
        T_term_mp_i = 0.5*0.5*[1 0 0 -1 0 0; ...
                            1 0 0 1 0 0; ...
                            1 0 0 -1 0 0; ...
                            1 0 0 1 0 0; ...
                            0 1 0 0 -1 0; ...
                            0 1 0 0 1 0; ...
                            0 1 0 0 -1 0; ...
                            0 1 0 0 1 0; ...
                            0 0 1 0 0 -1; ...
                            0 0 1 0 0 1; ...
                            0 0 1 0 0 -1; ...
                            0 0 1 0 0 1];
        T_m_mp = zeros(m,m);
        for row = 1:m
            T_m_mp(row, abs(map_case(row))) = sign(map_case(row));
        end
        T_mp_m = T_m_mp^(-1);
        T_m_mp = 0.5*[1 0 0 0 0 0; ...
                    0 1 0 0 0 0; ...
                    1 0 0 0 0 0; ...
                    0 1 0 0 0 0; ...
                    0 0 1 0 0 0; ...
                    0 0 0 1 0 0; ...
                    0 0 1 0 0 0; ...
                    0 0 0 1 0 0; ...
                    0 0 0 0 1 0; ...
                    0 0 0 0 0 1; ...
                    0 0 0 0 1 0; ...
                    0 0 0 0 0 1] * T_m_mp;
        T_mp_m = T_mp_m*2*[1 0 0 0 0 0 0 0 0 0 0 0; ...
                            0 1 0 0 0 0 0 0 0 0 0 0; ...
                            0 0 0 0 1 0 0 0 0 0 0 0; ...
                            0 0 0 0 0 1 0 0 0 0 0 0; ...
                            0 0 0 0 0 0 0 0 1 0 0 0; ...
                            0 0 0 0 0 0 0 0 0 1 0 0];
    
    case "Separate"
        T_term_ph_i = eye(m,m);
        T_ph_term_v = eye(m,m);
        Cm = 2/3;
        alpha = 2*pi/3;
        Cr = Cm*[cos(0*alpha) cos(1*alpha) cos(2*alpha) 0 0 0;
                 sin(0*alpha) sin(1*alpha) sin(2*alpha) 0 0 0;
                 0 0 0 cos(0*alpha) cos(1*alpha) cos(2*alpha);
                 0 0 0 sin(0*alpha) sin(1*alpha) sin(2*alpha);
                 1 1 1 0 0 0;
                 0 0 0 1 1 1];
        T_mp_term_v = eye(m,m);
        T_term_mp_v = T_mp_term_v^(-1);
        T_mp_term_i = eye(m,m);
        T_term_mp_i = T_mp_term_i^(-1);
        T_m_mp = eye(m,m);
        T_mp_m = T_m_mp^(-1);
        % Calculate Lmat for Separate configuration
        Lmat = 2/3*[Lt*[[1 cos(alpha) cos(2*alpha)]; [cos(alpha) 1 cos(alpha)]; [cos(2*alpha) cos(alpha) 1]] zeros(3,3);
                    zeros(3, 3) Ls*[[1 cos(alpha) cos(2*alpha)]; [cos(alpha) 1 cos(alpha)]; [cos(2*alpha) cos(alpha) 1]]] ...
                    + Llkg*eye(6);
        
        % Need to update Lt and Ls to include Llkg 
        L_alpha_beta = Cr*Lmat/Cr;
        Lt = L_alpha_beta(1,1);
        Ls = L_alpha_beta(3,3);
        tau_hat = 2*tau_hat;
        Fx_hat = 2*Fx_hat;
        Fy_hat = 2*Fy_hat;
end

%% Power electronics
Fsw = 88000; % Inverter switching frequency (Hz)
Tsw = 1/Fsw; % Inverter switching period (sec)

%% Controller
fb = 500; % Bandwidth (Hz)
wb = 2*pi*fb; % Bandwidth (rad/s)

Kp_t = Lt*wb; % Torque current regulation P gain
Ki_t = R*wb; % Torque current regulation I gain

Kp_s = Ls*wb; % Suspension current regulation P gain
Ki_s = R*wb; % Suspension current regulation I gain

%% Run simulation
out = sim('BearinglessMotorSimulation.slx');

%% Post processing
% Extract simulation data 
runObj = Simulink.sdi.Run.getLatest;

% List of variables to extract
obj2ext = {'time','vd','vq','id_cmd','iq_cmd','id','iq','torque_cmd','torque', ...
                  'vx','vy','ix_cmd','iy_cmd','ix','iy','fx_cmd','fy_cmd','fx','fy', ...
                  'i_term', 'v_term', ...
                  'EMF_1', 'EMF_2', 'EMF_3', 'EMF_4', 'EMF_5', 'EMF_6'};

% Get signal IDs and store signals into array
for idx = 2:length(obj2ext)
    sigID = getSignalIDsByName(runObj,obj2ext{idx});
    sig_obj.(obj2ext{idx}) = Simulink.sdi.getSignal(sigID);
    sig_val.(obj2ext{idx}) = sig_obj.(obj2ext{idx}).Values.Data;
end

time = sig_obj.(obj2ext{2}).Values.Time;

%% Plot figure
width = 2*5.43; 
height = 1.2*3*4.38 / 3;
set(0,'units','inches');
Inch_SS = get(0,'screensize');
lw = 1;  % line width

figure1 = figure;
% Plot terminal current
subplot(3,2,1);
hold on;
colors = lines(6); % Generate 6 types of colors
for i = 1:6 % Columns 1-6 contain current data
    plot(time, squeeze(sig_val.i_term(:,i)), 'Color', colors(i,:), 'LineWidth', lw);
end
xlabel('Time [s]','Interpreter','latex');
ylabel('Terminal current (A)','Interpreter','latex');
legend('Term 1','Term 2','Term 3','Term 4','Term 5','Term 6','Interpreter','latex','Location','east');
xlim([0 Tend]);
% ylim([-2.5 2.5]);

% Plot back-emf
subplot(3,2,2);
% Plot phase voltage
hold on;
for i = 1:6 % Columns 1-6 contain current data
    plot(time, squeeze(sig_val.v_term(i,:)), 'Color', colors(i,:), 'LineWidth', lw);
end
xlabel('Time [s]','Interpreter','latex');
ylabel('Terminal voltage (V)','Interpreter','latex');
legend('Term 1','Term 2','Term 3','Term 4','Term 5','Term 6','Interpreter','latex','Location','east');
xlim([0 Tend]);
% ylim([-2.5 2.5]);

% Plot torque and force
subplot(3,2,3);
hold on;
plot(time, squeeze(sig_val.id_cmd), 'Color', [231/255, 143/255, 106/255], 'LineWidth', lw);
plot(time, squeeze(sig_val.iq_cmd), 'Color', [0/255, 108/255, 186/255], 'LineWidth', lw);
plot(time, squeeze(sig_val.id), 'Color', [136/255, 62/255, 150/255], 'LineWidth', lw);
plot(time, squeeze(sig_val.iq), 'Color', [237/255, 170/255, 33/255], 'LineWidth', lw);
xlabel('Time [s]','Interpreter','latex');
ylabel('Torque current (A)','Interpreter','latex');
legend('$i_d^*$', '$i_q^*$', '$i_d$', '$i_q$','Interpreter','latex','Location','east');
xlim([0 Tend]);
% ylim([-2.5 2.5]);

subplot(3,2,4);
hold on;
plot(time, squeeze(sig_val.ix_cmd), 'Color', [231/255, 143/255, 106/255], 'LineWidth', lw);
plot(time, squeeze(sig_val.iy_cmd), 'Color', [0/255, 108/255, 186/255], 'LineWidth', lw);
plot(time, squeeze(sig_val.ix), 'Color', [136/255, 62/255, 150/255], 'LineWidth', lw);
plot(time, squeeze(sig_val.iy), 'Color', [237/255, 170/255, 33/255], 'LineWidth', lw);
xlabel('Time [s]','Interpreter','latex');
ylabel('Suspension current (A)','Interpreter','latex');
legend('$i_x^*$', '$i_y^*$', '$i_x$', '$i_y$','Interpreter','latex','Location','east');
xlim([0 Tend]);
% ylim([-2.5 2.5]);

subplot(3,2,5);
hold on;
plot(time, squeeze(sig_val.torque), 'Color', 'k', 'LineWidth', lw);
xlabel('Time [s]','Interpreter','latex');
ylabel('Torque (Nm)','Interpreter','latex');
legend('Torque','Interpreter','latex','Location','east');
xlim([0 Tend]);
% ylim([-2.5 2.5]);

subplot(3,2,6);
hold on;
plot(time, squeeze(sig_val.fx), 'Color', 'r', 'LineWidth', lw);
plot(time, squeeze(sig_val.fy), 'Color', 'b', 'LineWidth', lw);
xlabel('Time [s]','Interpreter','latex');
ylabel('Force (N)','Interpreter','latex');
legend('$f_x$','$f_y$','Interpreter','latex','Location','east');
xlim([0 Tend]);
% ylim([-2.5 2.5]);

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

set(figure1,'Units','inches','Position',[(Inch_SS(3)-width)/2 (Inch_SS(4)-height)/2 width height]);
print(figure1, '-dsvg','-noui','plot_results');
print(figure1, '-dpng','-r300','plot_results');

%% save data

% data.time = time;
% data.i_term1 = squeeze(sig_val.i_term(:,1));
% data.i_term4 = squeeze(sig_val.i_term(:,4));
% data.v_term1 = squeeze(sig_val.v_term(1,:)).';
% data.v_term4 = squeeze(sig_val.v_term(4,:)).';
% data.iq_cmd = squeeze(sig_val.iq_cmd);
% data.iq = squeeze(sig_val.iq);
% data.ix_cmd = squeeze(sig_val.ix_cmd);
% data.ix = squeeze(sig_val.ix);
% data.iy_cmd = squeeze(sig_val.iy_cmd);
% data.iy = squeeze(sig_val.iy);
% data.fx_cmd = squeeze(sig_val.fx_cmd);
% data.fx = squeeze(sig_val.fx);
% data.fy_cmd = squeeze(sig_val.fy_cmd);
% data.fy = squeeze(sig_val.fy);
% data.torque_cmd = squeeze(sig_val.torque_cmd);
% data.torque = squeeze(sig_val.torque);
% 
% data_table = table(data.time, data.i_term1, data.i_term4, data.v_term1, data.v_term4, ...
%     data.iq_cmd, data.iq, data.ix_cmd, data.ix, data.iy_cmd, data.iy, data.fx_cmd, data.fx, data.fy_cmd, data.fy,data.torque_cmd, data.torque);
% data_table.Properties.VariableNames = {'time', 'i_term1', 'i_term4', 'v_term1', 'v_term4', ...
%     'iq_cmd', 'iq', 'ix_cmd', 'ix', 'iy_cmd', 'iy', 'fx_cmd', 'fx', 'fy_cmd', 'fy', 'torque_cmd', 'torque'};
% 
% file_name = sprintf('../../../Publication/ISMB2025/full-paper/imaegs/simulink-results/%s_%s.csv', winding_configuration, conf);
% writetable(data_table, file_name);