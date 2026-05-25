D=0.5*rho0*1.3*Vs*S_ref*CD0;
T=3000;
W=m*g;
gamma=asind((T-D)/W);
% =========================================================================
% 无人机起飞与爬升性能计算脚本 (基于 Cessna 缩比无人机参数)
% =========================================================================
clc; clear; close all;

%% 1. 基础物理与气动参数 (从提供代码中提取)
m = 600.0;            % 飞机质量 (kg)
g = 9.80665;          % 重力加速度 (m/s^2)
W = m * g;            % 飞机重量 (N)

S_ref = 1.2;          % 机翼参考面积 (m^2)
rho0 = 1.225;         % 海平面空气密度 (kg/m^3)

% 气动参数
% 假设起飞最大升力系数由 CLa 和 CLq 叠加估算 (仅为示例，实际应取起飞构型的 CLmax)
CLa = 2.50;
CLq = 3.00;
CLmax = CLa + CLq;    % 估算最大升力系数 (5.5)

CD0 = 0.045;          % 零升阻力系数
K = 0.080;            % 诱导阻力因子

% 动力参数
Tmax = 3000.0;        % 最大推力 (N)

%% 2. 计算失速速度 (Vs) 与 起飞速度 (V_TO)
% -------------------------------------------------------------------------
% 失速速度公式: Vs = sqrt( 2*W / (rho * S * CLmax) )
Vs = sqrt((2 * W) / (rho0 * S_ref * CLmax));

% 工程上通常取起飞速度为失速速度的 1.2 倍
V_TO = 1.2 * Vs;

fprintf('--- 速度计算结果 ---\n');
fprintf('估算失速速度 (Vs)   = %.2f m/s (约 %.1f km/h)\n', Vs, Vs*3.6);
fprintf('安全起飞速度 (V_TO) = %.2f m/s (约 %.1f km/h)\n\n', V_TO, V_TO*3.6);

%% 3. 爬升性能分析 (扫描不同速度下的爬升率和爬升角)
% -------------------------------------------------------------------------
% 定义速度范围 (从起飞速度到 50m/s)
V_range = linspace(V_TO, 200, 100); 

% 预分配数组
gamma_deg = zeros(size(V_range)); % 爬升角 (度)
Rc = zeros(size(V_range));        % 爬升率 (m/s)
D_array = zeros(size(V_range));   % 阻力 (N)

for i = 1:length(V_range)
    V = V_range(i);
    
    % 1. 计算当前速度下的需用升力系数 (假设小角度爬升 L ≈ W)
    % W = 0.5 * rho * V^2 * S * CL  =>  CL = 2W / (rho * V^2 * S)
    CL_req = (2 * W) / (rho0 * V^2 * S_ref);
    
    % 2. 计算当前阻力系数和阻力
    CD_current = CD0 + K * CL_req^2;
    D = 0.5 * rho0 * V^2 * S_ref * CD_current;
    D_array(i) = D;
    
    % 3. 检查推力是否足够
    if Tmax >= D
        % 计算最大爬升角 (sin(gamma) = (T - D) / W)
        sin_gamma = (Tmax - D) / W;
        % 限制范围防数值越界
        sin_gamma = min(max(sin_gamma, -1), 1); 
        gamma_rad = asin(sin_gamma);
        
        % 转换为角度
        gamma_deg(i) = gamma_rad * (180 / pi);
        
        % 计算最大爬升率 (Rc = V * sin(gamma))
        Rc(i) = V * sin_gamma;
    else
        % 推力不足以克服阻力，无法爬升
        gamma_deg(i) = NaN;
        Rc(i) = NaN;
    end
end

% 寻找最佳爬升点
[max_gamma, idx_gamma] = max(gamma_deg);
Vx = V_range(idx_gamma); % 最大爬升角速度

[max_Rc, idx_Rc] = max(Rc);
Vy = V_range(idx_Rc);    % 最大爬升率速度

fprintf('--- 爬升性能结果 (海平面, 最大推力) ---\n');
fprintf('最大爬升角 (gamma_max) = %.2f 度, 对应的最佳速度 (Vx) = %.2f m/s\n', max_gamma, Vx);
fprintf('最大爬升率 (Rc_max)    = %.2f m/s, 对应的最佳速度 (Vy) = %.2f m/s\n\n', max_Rc, Vy);

%% 4. 绘制性能曲线
% -------------------------------------------------------------------------
figure('Name', '无人机爬升性能分析', 'Position', [100, 100, 900, 400]);

% 图1: 爬升角 vs 速度
subplot(1, 2, 1);
plot(V_range, gamma_deg, 'b-', 'LineWidth', 2);
hold on; grid on;
plot(Vx, max_gamma, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(Vx+1, max_gamma, sprintf('最大爬升角: %.1f°\n@Vx=%.1fm/s', max_gamma, Vx));
title('最大爬升角随速度变化曲线');
xlabel('飞行速度 V (m/s)');
ylabel('最大爬升角 \gamma (度)');
xlim([V_TO, 200]);

% 图2: 爬升率 vs 速度
subplot(1, 2, 2);
plot(V_range, Rc, 'g-', 'LineWidth', 2);
hold on; grid on;
plot(Vy, max_Rc, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(Vy+1, max_Rc, sprintf('最大爬升率: %.1fm/s\n@Vy=%.1fm/s', max_Rc, Vy));
title('最大爬升率随速度变化曲线');
xlabel('飞行速度 V (m/s)');
ylabel('最大爬升率 Rc (m/s)');
xlim([V_TO, 200]);

sgtitle('无人机动力学爬升性能分析 (海平面状态)');