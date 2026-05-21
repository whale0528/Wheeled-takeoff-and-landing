% plot_trajectory.m — 飞行器轨迹可视化
% 使用前请在 MATLAB 中先运行 plan.m 初始化参数，然后运行本脚本
% 用法: run('plan'); plot_trajectory;

%% 0. 检查工作区变量
if ~exist('HD', 'var') || ~exist('K_alpha', 'var')
    fprintf('Workspace not initialized. Running plan + controller...\n');
    run('plan');
    run('controller');
end

close all;
load_system('planmodel');

%% 1. 配置信号记录 — 记录关键块输出端口
fprintf('Configuring signal logging...\n');

% 记录 6DOF 动力学中关键输出端口
% X/Y/Z 位置: Subsystem3 的输出端口
ports_to_log = {
    'planmodel/6DOF-MODEL/动力学/Subsystem3', 3;   % X_position
    'planmodel/6DOF-MODEL/动力学/Subsystem3', 4;   % Y_position
    'planmodel/6DOF-MODEL/动力学/Subsystem3', 5;   % Z_position
    'planmodel/6DOF-MODEL/动力学/Subsystem3', 1;   % alpha(H)
    'planmodel/6DOF-MODEL/动力学/Subsystem3', 2;   % beta(H)
    'planmodel/6DOF-MODEL/动力学/姿态角', 1;         % gamma
    'planmodel/6DOF-MODEL/动力学/姿态角', 2;         % theta
    'planmodel/6DOF-MODEL/动力学/姿态角', 3;         % psi
};

logged = 0;
for i = 1:size(ports_to_log, 1)
    try
        ph = get_param(ports_to_log{i, 1}, 'PortHandles');
        set_param(ph.Outport(ports_to_log{i, 2}), 'DataLogging', 'on');
        logged = logged + 1;
    catch
        fprintf('  WARNING: Could not log %s port %d\n', ports_to_log{i,1}, ports_to_log{i,2});
    end
end

% 记录控制子系统输出
try
    ph = get_param('planmodel/control', 'PortHandles');
    set_param(ph.Outport(1), 'DataLogging', 'on');  % de
    set_param(ph.Outport(2), 'DataLogging', 'on');  % dr
    set_param(ph.Outport(3), 'DataLogging', 'on');  % da
    logged = logged + 3;
end

fprintf('  Enabled logging on %d ports.\n', logged);

%% 2. 运行仿真
fprintf('Running simulation...\n');
simOut = sim('planmodel');
logsout = get(simOut, 'logsout');
tout = simOut.tout;
fprintf('Simulation completed (%.1f s).\n', tout(end));

%% 3. 提取信号 — 用 BlockPath + PortIndex 匹配
% Subsystem3 端口: 1=alpha, 2=beta, 3=X, 4=Y(高度), 5=Z(侧向)
% 姿态角 端口:    1=gamma, 2=theta, 3=psi
% control 端口:   1=de, 2=dr, 3=da

X = []; Y = []; Z = [];
gamma = []; theta = []; psi = [];
alpha = []; beta = [];
de = []; dr = []; da = [];

for i = 1:logsout.numElements
    el = logsout{i};
    blk_path = el.BlockPath.getBlock(1);
    pi = el.PortIndex;
    vals = el.Values;
    d = vals.Data(:); t = vals.Time(:);

    if contains(blk_path, 'Subsystem3')
        switch pi
            case 1, alpha = timeseries(d, t);
            case 2, beta  = timeseries(d, t);
            case 3, X     = timeseries(d, t);   % 前向
            case 4, Y     = timeseries(d, t);   % 高度 (坐标系 Y 向上)
            case 5, Z     = timeseries(d, t);   % 侧向
        end
    elseif contains(blk_path, '姿态角')
        switch pi
            case 1, gamma = timeseries(d, t);
            case 2, theta = timeseries(d, t);
            case 3, psi   = timeseries(d, t);
        end
    elseif strcmp(blk_path, 'planmodel/control')
        switch pi
            case 1, de = timeseries(d, t);
            case 2, dr = timeseries(d, t);
            case 3, da = timeseries(d, t);
        end
    end
end

%% 4. 降级匹配 — 找不到就用 GotoTag
if isempty(X), X = find_by_tag(logsout, 'X_positon'); end
if isempty(Y), Y = find_by_tag(logsout, 'Y_positon'); end
if isempty(Z), Z = find_by_tag(logsout, 'Z_positon'); end

% 数据检查
if isempty(X) || isempty(Y)
    error('Failed to extract position data. Ensure model has signal logging enabled.');
end

%% 5. 准备绘图数据 (坐标系: X=前, Y=上, Z=右)
hd = 57.2958;  % rad -> deg

x = X.Data;            % 前向距离 [m]
alt = Y.Data;          % 高度 = Y [m]
t_X = X.Time;  t_alt = Y.Time;
if ~isempty(Z)
    y_lat = Z.Data;   % 侧向
else
    y_lat = zeros(size(x));
end

% 速度估算 (X-Y 平面)
dt = tout(2) - tout(1);
dx = diff(x); dy = diff(y_lat);
Vg_est = sqrt(dx.^2 + dy.^2) / dt;
t_Vg = tout(1:end-1);

fprintf('Data ready for plotting.\n');

%% 6. Fig 1 — 3D 轨迹
figure('Name', '3D Trajectory', 'Position', [50, 300, 700, 500]);
plot3(x, y_lat, alt, 'b-', 'LineWidth', 1.2); hold on;
plot3(x(1), y_lat(1), alt(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(x(end), y_lat(end), alt(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X [m]'); ylabel('Z (lateral) [m]'); zlabel('Altitude [m]');
title('3D Flight Trajectory');
grid on; view(30, 20);
legend('Trajectory', 'Start', 'End', 'Location', 'best');

%% 7. Fig 2 — 综合仪表盘
figure('Name', 'Trajectory Dashboard', 'Position', [50, 50, 1200, 800]);

% (1) 高度
subplot(3,4,1);
plot(t_alt, alt, 'b-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Alt [m]'); title('Altitude'); grid on;

% (2) 地速 (估算)
subplot(3,4,2);
plot(t_Vg, Vg_est, 'b-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('V_g [m/s]'); title('Ground Speed (est.)'); grid on;

% (3) 俯仰角
subplot(3,4,3);
if ~isempty(theta)
    plot(theta.Time, theta.Data * hd, 'b-', 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('\theta [deg]'); title('Pitch Angle'); grid on;

% (4) 滚转角
subplot(3,4,4);
if ~isempty(gamma)
    plot(gamma.Time, gamma.Data * hd, 'b-', 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('\gamma [deg]'); title('Roll Angle'); grid on;

% (5) 地面轨迹
subplot(3,4,5);
plot(x, y_lat, 'b-', 'LineWidth', 1.2); hold on;
plot(x(1), y_lat(1), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
plot(x(end), y_lat(end), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
xlabel('X [m]'); ylabel('Z (lateral) [m]'); title('Ground Track'); grid on; axis equal;

% (6) 偏航角
subplot(3,4,6);
if ~isempty(psi)
    plot(psi.Time, psi.Data * hd, 'b-', 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('\psi [deg]'); title('Yaw Angle'); grid on;

% (7) 迎角
subplot(3,4,7);
if ~isempty(alpha)
    plot(alpha.Time, alpha.Data * hd, 'b-', 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('\alpha [deg]'); title('Angle of Attack'); grid on;

% (8) 侧滑角
subplot(3,4,8);
if ~isempty(beta)
    plot(beta.Time, beta.Data * hd, 'b-', 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('\beta [deg]'); title('Sideslip'); grid on;

% (9) 升降舵
subplot(3,4,9);
if ~isempty(de)
    plot(de.Time, de.Data, 'b-', 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('\delta_e [deg]'); title('Elevator'); grid on;

% (10) 方向舵
subplot(3,4,10);
if ~isempty(dr)
    plot(dr.Time, dr.Data, 'b-', 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('\delta_r [deg]'); title('Rudder'); grid on;

% (11) 副翼
subplot(3,4,11);
if ~isempty(da)
    plot(da.Time, da.Data, 'b-', 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('\delta_a [deg]'); title('Aileron'); grid on;

% (12) 迎角 vs 俯仰角 对比
subplot(3,4,12);
if ~isempty(alpha) && ~isempty(theta)
    plot(alpha.Time, alpha.Data * hd, 'b-', 'LineWidth', 1.2); hold on;
    plot(theta.Time, theta.Data * hd, 'r-', 'LineWidth', 1.0);
    xlabel('Time [s]'); ylabel('[deg]'); title('\alpha vs \theta'); grid on;
    legend('\alpha', '\theta', 'Location', 'best');
end

sgtitle('Flight Trajectory Dashboard');

%% 8. 起飞参数汇总
fprintf('\n========== Takeoff Summary ==========\n');

% 抬轮 (theta > 0.5 deg, theta 内部是弧度需转换)
if ~isempty(theta)
    th_d = theta.Data * hd; th_t = theta.Time;
    idx = find(th_d > 0.5, 1, 'first');
    if ~isempty(idx)
        fprintf('Rotation starts:    t = %.2f s, theta = %.2f deg\n', th_t(idx), th_d(idx));
    end
end

% 离地 (高度超过初始值 0.5m 以上)
alt_init = alt(1);
idx_lo = find(alt > alt_init + 0.5, 1, 'first');
if ~isempty(idx_lo)
    t_lo = t_alt(idx_lo);
    fprintf('Lift-off:           t = %.2f s, Alt = %.2f m\n', t_lo, alt(idx_lo));
    [~, vi] = min(abs(t_Vg - t_lo));
    fprintf('  Ground speed:     %.2f m/s\n', Vg_est(vi));
else
    fprintf('Lift-off: NOT ACHIEVED within %.0f s\n', tout(end));
end

% 最高点
[max_alt, idx_max] = max(alt);
fprintf('Max altitude:       %.2f m at t = %.2f s\n', max_alt, t_alt(idx_max));

% 终止状态
fprintf('\nFinal (t = %.1f s):\n', tout(end));
fprintf('  Pos:  X = %.0f m,  Alt = %.1f m\n', x(end), alt(end));
fprintf('  Speed: %.2f m/s\n', Vg_est(end));
if ~isempty(theta),  fprintf('  Pitch: %.2f deg\n', theta.Data(end) * hd); end
if ~isempty(alpha),  fprintf('  AoA:   %.2f deg\n', alpha.Data(end) * hd); end
fprintf('=====================================\n');

%% Helper: 从 logsout 按 GotoTag 查找信号
function ts = find_by_tag(logsout, tag)
    ts = [];
    for k = 1:logsout.numElements
        el = logsout{k};
        try
            blk = el.BlockPath.getBlock(1);
            if strcmp(get_param(blk, 'BlockType'), 'Goto') && ...
               strcmp(get_param(blk, 'GotoTag'), tag)
                vals = el.Values;
                ts = timeseries(vals.Data(:), vals.Time(:));
                return;
            end
        end
    end
end
