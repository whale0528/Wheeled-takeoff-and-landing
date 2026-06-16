
%% 飞机起飞初始值
Xg0 = -50000.0;            % m, 前向位置

Yg0 = 0.0;            % m, 右向位置，向右为正

u0_b = 200.0;           % m/s, 机体系前向速度
v0_b = 0.0;           % m/s, 机体系右向速度
w0_b = 0.0;           % m/s, 机体系下向速度

V_min = 3;            % 防止除0

phi0   = 0.0;         % rad, 滚转角，绕X轴
theta0 = 0.0;         % rad, 俯仰角，绕Y轴
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

%% 起飞阶段参考参数 — A320 数值
% VR 使用 §12 最小抬轮速度 61.73 m/s
VR = 61.733333;             % m/s, 抬轮速度 (来源: §12)
H_safe = 10.0;              % m, 安全爬升高度
Hdot_safe = 1.0;            % m/s, 安全爬升率
H_climb_complete = 800.0;   % m, 爬升完成高度
rotation_rate_cmd = 3.0 * DH;  % rad/s, 抬轮俯仰角速度指令
climb_eta_cmd = 6.0 * DH;      % rad, 初始爬升航迹倾角
X_landing_start = -11500;
% V_flare 使用 1.4 * 全襟翼失速速度 ≈82.8 m/s
V_flare = 1.4 * V_stall_full;
save("takeoff_parameter.mat");
