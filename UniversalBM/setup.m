clear
close all
addpath(genpath(pwd))

%% Switch winding that you develop
load_system('BearinglessMotorSimulation') % load Simulink model
load_system("Plant")
load_system("Controller")
% The input of this function should be 'Separate', 'MP', 'Bridge',
% 'Parallel', or 'MCI'
winding_configuration = "MP";

winding_conf_dic = dictionary("Separate", 1, ...
                              "MP", 2, ...
                              "MP2", 3, ...
                              "Bridge", 4, ...
                              "Parallel", 5, ...
                              "MCI", 6);
winding_conf_num = winding_conf_dic(winding_configuration);

Tend = 0.03; % Simulation stop time (s)
Tsim = 1e-5; % Simulation sampling time (s)
V_DC = 160; % DC bus (V)

tau_ref = 0.2; % Torque reference (Nm)
tau_start = (1/6)*Tend; % Torque start time (s)
tau_end = (4/6)*Tend; % Torque end time (s)

fx_ref = 9; % fx force reference (N)
fx_start = 0; % x-axis force start time (s)
fx_end = (2/6)*Tend; % x-axis force end time (s)
fy_ref = 9; % fy force reference (N)
fy_start = (3/6)*Tend; % y-axis force start time (s)
fy_end = (5/6)*Tend; % y-axis force end time (s)

id_ref = 0; % id current reference (A)
id_start = 0; % d-axis current start time (s)
id_end = 0.025; % d-axis current end time (s)

rotor_unbalance_weight = 1; % (kg)
rotor_unbalance_pos = 0; % epsilon (m)

speed_ref = 7500; % Speed command (r/min)

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

% Calculate Generalized Clarke Transform
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
Ke = 0.00667;  % back-emf constant (Vpk/(mech rad/s))
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
obj2ext = {'time','vd','vq','id_ref','iq_ref','id','iq','tau_ref','tau', ...
                  'vx','vy','ix_ref','iy_ref','ix','iy','fx_ref','fy_ref','fx','fy', ...
                  'i_term', 'v_term', ...
                  'EMF_1', 'EMF_2', 'EMF_3', 'EMF_4', 'EMF_5', 'EMF_6'};

% Get signal IDs and store signals into array
for idx = 2:length(obj2ext)
    sigID = getSignalIDsByName(runObj,obj2ext{idx});
    sig_obj.(obj2ext{idx}) = Simulink.sdi.getSignal(sigID);
    sig_val.(obj2ext{idx}) = sig_obj.(obj2ext{idx}).Values.Data;
end

time = sig_obj.(obj2ext{2}).Values.Time;
time = time*1e3;  % Convert to msec
Tmax = Tend*1e3;

%% Plot figure
width = 5.43*0.7; 
height = 1.2*3*4.38*2 / 3*0.7;
set(0,'units','inches');
Inch_SS = get(0,'screensize');
lw = 1;  % line width

figure1 = figure;
% Plot torque
subplot(6,1,1);
hold on;
plot(time, squeeze(sig_val.tau_ref), 'LineStyle', '--', 'Color', 'b', 'LineWidth', lw);
plot(time, squeeze(sig_val.tau), 'Color', 'b', 'LineWidth', lw);
xlabel('Time [s]','Interpreter','latex');
ylabel('$\tau$ (Nm)','Interpreter','latex');
legend('$\tau^{\mathrm{ref}}$', '$\tau$','Interpreter','latex','Location','east');
xlim([0 Tmax]);
% ylim([-2.5 2.5]);

subplot(6,1,2);
% Plot forces
hold on;
plot(time, squeeze(sig_val.fx_ref), 'LineStyle', '--', 'Color', 'r', 'LineWidth', lw);
plot(time, squeeze(sig_val.fx), 'Color', 'r', 'LineWidth', lw);
plot(time, squeeze(sig_val.fy_ref), 'LineStyle', '--', 'Color', 'g', 'LineWidth', lw);
plot(time, squeeze(sig_val.fy), 'Color', 'g', 'LineWidth', lw);
xlabel('Time [s]','Interpreter','latex');
ylabel('$F_{\mathrm{x}}$, $F_{\mathrm{y}}$ (N)','Interpreter','latex');
legend('$F_\mathrm{x}^\mathrm{ref}$','$F_{\mathrm{x}}$','$F_\mathrm{y}^\mathrm{ref}$','$F_{\mathrm{y}}$','Interpreter','latex','Location','east');
xlim([0 Tmax]);
% ylim([-2.5 2.5]);

% Plot torque current
subplot(6,1,3);
hold on;
% plot(time, squeeze(sig_val.id_ref), 'Color', [231/255, 143/255, 106/255], 'LineWidth', lw);
plot(time, squeeze(sig_val.iq_ref), 'LineStyle', '--', 'Color', 'b', 'LineWidth', lw);
% plot(time, squeeze(sig_val.id), 'Color', [136/255, 62/255, 150/255], 'LineWidth', lw);
plot(time, squeeze(sig_val.iq), 'Color', 'b', 'LineWidth', lw);
xlabel('Time [s]','Interpreter','latex');
ylabel('$\vec{i}_\mathrm{t}^{\,\, \rm T}$ (A)','Interpreter','latex');
legend('$i_\mathrm{q}^\mathrm{ref}$','$i_\mathrm{q}$','Interpreter','latex','Location','east');
xlim([0 Tmax]);
% ylim([-2.5 2.5]);

% Plot suspension current
subplot(6,1,4);
hold on;
plot(time, squeeze(sig_val.ix_ref), 'LineStyle', '--', 'Color', 'r', 'LineWidth', lw);
plot(time, squeeze(sig_val.ix), 'Color', 'r', 'LineWidth', lw);
plot(time, squeeze(sig_val.iy_ref), 'LineStyle', '--', 'Color', 'g', 'LineWidth', lw);
plot(time, squeeze(sig_val.iy), 'Color', 'g', 'LineWidth', lw);
xlabel('Time [s]','Interpreter','latex');
ylabel('$\vec{i}_\mathrm{s}^{\,\, \rm S}$ (A)','Interpreter','latex');
legend('$i_\mathrm{x}^\mathrm{ref}$', '$i_\mathrm{x}$', '$i_\mathrm{y}^\mathrm{ref}$', '$i_\mathrm{y}$','Interpreter','latex','Location','east');
xlim([0 Tmax]);
% ylim([-2.5 2.5]);

% Plot terminal current
subplot(6,1,5);
hold on;
colors = lines(6); % Generate 6 types of colors        
colors(1,:) = [0 0 1];
colors(4,:) = [1 0 0];
for i = [1, 4] % Columns 1-6 contain current data
    plot(time, squeeze(sig_val.i_term(:,i)), 'Color', colors(i,:), 'LineWidth', lw);
end
xlabel('Time [s]','Interpreter','latex');
ylabel('$\mathbf{i}_{\mathrm{term}}$ (A)','Interpreter','latex');
legend('$I_{u}$','$I_{u^\prime}$','Interpreter','latex','Location','east');
xlim([0 Tmax]);
% ylim([-2.5 2.5]);

% Plot terminal voltage
subplot(6,1,6);
hold on;
for i = [1, 4] % Columns 1-6 contain current data
    plot(time, squeeze(sig_val.v_term(i,:)), 'Color', colors(i,:), 'LineWidth', lw);
end
xlabel('Time (ms)','Interpreter','latex');
ylabel('$\mathbf{v}_{\mathrm{term}}$ (V)','Interpreter','latex');
legend('$v_{u}$','$v_{u^\prime}$','Interpreter','latex','Location','east');
xlim([0 Tmax]);
% ylim([-2.5 2.5]);

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

set(figure1,'Units','inches','Position',[(Inch_SS(3)-width)/2 (Inch_SS(4)-height)/2 width height]);
