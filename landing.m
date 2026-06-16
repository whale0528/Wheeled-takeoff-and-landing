
basedata;
load("control_design.mat");

Zg0 = -1000;
%% 飞机着陆初始值
Xg0 = -11500.0;            % m, 前向位置
Yg0 = 100.0;               % m, 右向位置，向右为正

% 进近速度 = 1.4 * 全襟翼失速速度 (来源: §12)
u0_b = 1.4 * V_stall_full; % m/s ≈82.8, 机体系前向速度
v0_b = 0.0;                % m/s, 机体系右向速度
w0_b = 0.0;                % m/s, 机体系下向速度

V_min = 3;            % 防止除0

phi0   = 0.0;         % rad, 滚转角，绕X轴
theta0 = alpha_trim;  % rad, 用配平迎角作为初始俯仰角
psi0   = 0.0;         % rad, 偏航角，绕Z轴

p0 = 0.0;             % rad/s, 绕X滚转角速度
q0 = 0.0;             % rad/s, 绕Y俯仰角速度
r0 = 0.0;             % rad/s, 绕Z偏航角速度

x0_takeoff = [Xg0;
              Yg0;
              Zg0;
              u0_b;
              v0_b;
              w0_b;
              phi0;
              theta0;
              psi0;
              p0;
              q0;
              r0];

%% 起飞阶段参考参数
H_safe = 10.0;              % m, 安全爬升高度
Hdot_safe = 1.0;            % m/s, 安全爬升率
H_climb_complete = 1000.0;  % m, 爬升完成高度

rotation_rate_cmd = 3.0 * DH;  % rad/s, 抬轮俯仰角速度指令
climb_eta_cmd = 6.0 * DH;      % rad, 初始爬升航迹倾角

%% 配平参考值 — 从 control_design.m 计算结果
D0 = D0_val;             % N, 配平阻力
L0 = m * g;              % N, 配平升力

% 配平升降舵偏角 — 从 Cm 表配平条件反算
% Cm_base(alpha_trim) + Cmde * K_δe(α_trim) * delta_e = 0
Cm_base_trim = interp1(alpha_Cm_table, Cm_table, alpha_trim, 'pchip');
Kde_trim = interp1(alpha_Kde_table, Kde_table, alpha_trim, 'pchip');
delta_p0 = -Cm_base_trim / (s_mdeltae * Cmde * Kde_trim) * HD;  % deg

% 爬升性能
gamma_climbmax = asin((thrust_max - D0) / (m * g)) * HD;  % 最大爬升角 (deg)
VR = VR_min;              % m/s, 抬轮速度
X_landing_start = -11500;
V_flare = 1.4 * V_stall_full;  % m/s ≈82.8
save("landingplan.mat");
