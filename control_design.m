%% 控制器增益设计 — A320 双设计点
% 设计点1: 巡航 V=234.07 m/s
% 设计点2: 起飞/着陆 V=118.84 m/s
% CLa/Cma 从 CL/Cm 插值表局部斜率提取（不再使用硬编码单值）

load("basedata.mat");

% 带宽和阻尼设置
wd_pitch   = 15;
kesi_pitch = 0.8;
Tr         = 1.2;
kesi_yaw   = 0.8;
Tr_yaw     = 1.2;
wd_yaw     = 15;

% 有限差分扰动
d_alpha = 0.01;  % rad

% 预处理：取 CL 表预失速单调区间，用于从 CL 反查 α
% CL_table 在 α≈0.26 rad 达到峰值 1.76，之后进入失速区递减
% 反查只能在单调递增段进行
[~, idx_cl_peak] = max(CL_table);
CL_table_mono = CL_table(1:idx_cl_peak);
alpha_mono    = alpha_CL_table(1:idx_cl_peak);

% 辅助函数：从 CL 反查 α
alpha_from_CL = @(CL_val) interp1(CL_table_mono, alpha_mono, CL_val, 'pchip');

%% ===== 设计点1: 巡航 =====
V_cruise = 234.072222;  % m/s (来源: §12)
qc_cruise = 0.5 * rho0 * V_cruise^2;

% 配平 CL 和 α
CL_trim_cruise = m * g / (qc_cruise * S_ref);
alpha_trim_cruise = alpha_from_CL(CL_trim_cruise);

% 升降舵效能倍率 @ 配平迎角 (仅用于 Cm!)
Kde_cruise = interp1(alpha_Kde_table, Kde_table, alpha_trim_cruise, 'pchip');
Cmde_eff_cruise = s_mdeltae * Cmde * Kde_cruise;
CLde_eff_cruise = s_Ldeltae * CLde;  % CL 不乘 K_δe

% 从插值表提取局部斜率（中心差分）
CL_plus  = interp1(alpha_CL_table, CL_table, alpha_trim_cruise + d_alpha, 'pchip');
CL_minus = interp1(alpha_CL_table, CL_table, alpha_trim_cruise - d_alpha, 'pchip');
CLa_cruise = (CL_plus - CL_minus) / (2 * d_alpha);

Cm_plus  = interp1(alpha_Cm_table, Cm_table, alpha_trim_cruise + d_alpha, 'pchip');
Cm_minus = interp1(alpha_Cm_table, Cm_table, alpha_trim_cruise - d_alpha, 'pchip');
Cma_cruise = (Cm_plus - Cm_minus) / (2 * d_alpha);

% 纵向通道 (s_* 标定系数乘在对应导数上)
a24_cruise = -s_malpha * Cma_cruise * qc_cruise * S_ref * c_ref / Iyy;
a25_cruise = -Cmde_eff_cruise * qc_cruise * S_ref * c_ref / Iyy;
a34_cruise = CLa_cruise * qc_cruise * S_ref / (m * V_cruise);

Kw_pitch_cruise = -2 * kesi_pitch * wd_pitch / a25_cruise;
K_alpha_cruise  = (wd_pitch^2 - a24_cruise) / (2 * kesi_pitch * wd_pitch);
Kn_pitch_cruise = -(2.2 * g * wd_pitch^2) / (Tr * (wd_pitch^2 - a24_cruise) * V_cruise * a34_cruise);

% 偏航通道
D0_cruise = qc_cruise * S_ref * (CD0 + K * (CL_trim_cruise - CL_D0)^2);

b24_cruise = -s_nbeta * Cnb * qc_cruise * S_ref * b_ref / Izz;
b27_cruise = -s_ndeltar * Cndr * qc_cruise * S_ref * b_ref / Izz;
b34_cruise = (D0_cruise - s_Ybeta * CSb * qc_cruise * S_ref) / (m * V_cruise);
b37_cruise = -s_Ydeltar * CSdr * qc_cruise * S_ref / (m * V_cruise);

Kw_yaw_cruise = -2 * kesi_yaw * wd_yaw / b27_cruise;
K_beta_cruise = (wd_yaw^2 - b24_cruise) / (2 * kesi_yaw * wd_yaw);
Kn_yaw_cruise = (2.2 * g * wd_yaw^2) / (Tr_yaw * (wd_yaw^2 - b24_cruise) * V_cruise * b34_cruise);

%% ===== 设计点2: 起飞/着陆 =====
V_to = 118.836667;  % m/s (来源: §12, 爬升速度)
qc_to = 0.5 * rho0 * V_to^2;

% 配平 CL 和 α
CL_trim_to = m * g / (qc_to * S_ref);
alpha_trim_to = alpha_from_CL(CL_trim_to);

% 升降舵效能倍率 @ 配平迎角
Kde_to = interp1(alpha_Kde_table, Kde_table, alpha_trim_to, 'pchip');
Cmde_eff_to = s_mdeltae * Cmde * Kde_to;  % K_δe 仅用于 Cm
CLde_eff_to = s_Ldeltae * CLde;            % CL 不乘 K_δe

% 从插值表提取局部斜率
CL_plus  = interp1(alpha_CL_table, CL_table, alpha_trim_to + d_alpha, 'pchip');
CL_minus = interp1(alpha_CL_table, CL_table, alpha_trim_to - d_alpha, 'pchip');
CLa_to = (CL_plus - CL_minus) / (2 * d_alpha);

Cm_plus  = interp1(alpha_Cm_table, Cm_table, alpha_trim_to + d_alpha, 'pchip');
Cm_minus = interp1(alpha_Cm_table, Cm_table, alpha_trim_to - d_alpha, 'pchip');
Cma_to = (Cm_plus - Cm_minus) / (2 * d_alpha);

% 纵向通道
a24_to = -s_malpha * Cma_to * qc_to * S_ref * c_ref / Iyy;
a25_to = -Cmde_eff_to * qc_to * S_ref * c_ref / Iyy;
a34_to = CLa_to * qc_to * S_ref / (m * V_to);

Kw_pitch_to = -2 * kesi_pitch * wd_pitch / a25_to;
K_alpha_to  = (wd_pitch^2 - a24_to) / (2 * kesi_pitch * wd_pitch);
Kn_pitch_to = -(2.2 * g * wd_pitch^2) / (Tr * (wd_pitch^2 - a24_to) * V_to * a34_to);

% 偏航通道
D0_to = qc_to * S_ref * (CD0 + K * (CL_trim_to - CL_D0)^2);

b24_to = -s_nbeta * Cnb * qc_to * S_ref * b_ref / Izz;
b27_to = -s_ndeltar * Cndr * qc_to * S_ref * b_ref / Izz;
b34_to = (D0_to - s_Ybeta * CSb * qc_to * S_ref) / (m * V_to);
b37_to = -s_Ydeltar * CSdr * qc_to * S_ref / (m * V_to);

Kw_yaw_to = -2 * kesi_yaw * wd_yaw / b27_to;
K_beta_to = (wd_yaw^2 - b24_to) / (2 * kesi_yaw * wd_yaw);
Kn_yaw_to = (2.2 * g * wd_yaw^2) / (Tr_yaw * (wd_yaw^2 - b24_to) * V_to * b34_to);

%% ===== 输出 =====
% 巡航增益（保持原变量名兼容 Simulink）
Kw_pitch = Kw_pitch_cruise;
K_alpha  = K_alpha_cruise;
Kn_pitch = Kn_pitch_cruise;
Kw_yaw   = Kw_yaw_cruise;
K_beta   = K_beta_cruise;
Kn_yaw   = Kn_yaw_cruise;

% 起飞/着陆增益
Kw_pitch_to_saved = Kw_pitch_to;
K_alpha_to_saved  = K_alpha_to;
Kn_pitch_to_saved = Kn_pitch_to;
Kw_yaw_to_saved   = Kw_yaw_to;
K_beta_to_saved   = K_beta_to;
Kn_yaw_to_saved   = Kn_yaw_to;

% 配平信息（供 landing.m 使用）
alpha_trim = alpha_trim_to;
CL_trim_val = CL_trim_to;
D0_val = D0_to;

fprintf('===== 巡航设计点 (V=%.1f m/s) =====\n', V_cruise);
fprintf('  动压 qc = %.0f Pa\n', qc_cruise);
fprintf('  配平 CL = %.4f, alpha = %.4f rad (%.2f deg)\n', CL_trim_cruise, alpha_trim_cruise, alpha_trim_cruise*180/pi);
fprintf('  CLa_local = %.4f, Cma_local = %.4f\n', CLa_cruise, Cma_cruise);
fprintf('  Kw_pitch=%.6f, K_alpha=%.6f, Kn_pitch=%.6f\n', Kw_pitch_cruise, K_alpha_cruise, Kn_pitch_cruise);
fprintf('  Kw_yaw=%.6f, K_beta=%.6f, Kn_yaw=%.6f\n', Kw_yaw_cruise, K_beta_cruise, Kn_yaw_cruise);

fprintf('\n===== 起飞/着陆设计点 (V=%.1f m/s) =====\n', V_to);
fprintf('  动压 qc = %.0f Pa\n', qc_to);
fprintf('  配平 CL = %.4f, alpha = %.4f rad (%.2f deg)\n', CL_trim_to, alpha_trim_to, alpha_trim_to*180/pi);
fprintf('  CLa_local = %.4f, Cma_local = %.4f\n', CLa_to, Cma_to);
fprintf('  Kw_pitch=%.6f, K_alpha=%.6f, Kn_pitch=%.6f\n', Kw_pitch_to, K_alpha_to, Kn_pitch_to);
fprintf('  Kw_yaw=%.6f, K_beta=%.6f, Kn_yaw=%.6f\n', Kw_yaw_to, K_beta_to, Kn_yaw_to);

save("control_design.mat", ...
    "Kw_pitch", "K_alpha", "Kn_pitch", "Kw_yaw", "K_beta", "Kn_yaw", ...
    "Kw_pitch_to_saved", "K_alpha_to_saved", "Kn_pitch_to_saved", ...
    "Kw_yaw_to_saved", "K_beta_to_saved", "Kn_yaw_to_saved", ...
    "alpha_trim", "CL_trim_val", "D0_val", ...
    "V_cruise", "V_to", "alpha_trim_cruise", "alpha_trim_to");
