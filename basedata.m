%% 数据来源：《基于A320气动数据的无人机自主起降巡航完整仿真方案》
%% 所有 § 引用均指向该文档对应章节
%%
%% 数学运算
HD = 180 / pi;        % rad -> deg
DH = pi / 180;        % deg -> rad

%% 坐标系说明
% 机体系/地面系均采用：
% X 轴：向前
% Y 轴：向右
% Z 轴：向下
%
% 状态量定义：
% x = [Xg; Yg; Zg; u; v; w; phi; theta; psi; p; q; r]

%% 飞机基本参数 — A320 空机 (来源: §14.1)
m = 42500.244;        % kg
g = 9.80665;          % m/s^2

S_ref = 122.396968;   % m^2  (来源: §4)
b_ref = 35.799979;    % m    (来源: §4)
c_ref = S_ref / b_ref;% m    (来源: §4.1, S/b 代理)

%% 转动惯量 (来源: §14.1)
% 按标准机体系映射: Ixx滚转, Iyy俯仰, Izz偏航
Ixx = 1340910.730;    % kg*m^2, 绕 X 轴滚转惯量
Iyy = 3326789.481;    % kg*m^2, 绕 Y 轴俯仰惯量
Izz = 4292706.727;    % kg*m^2, 绕 Z 轴偏航惯量

Ixy = 0.0;
Ixz = 1355.818;       % kg*m^2, 耦合惯量 (来源: §14.1)
Iyz = 0.0;

I = [ Ixx, -Ixy, -Ixz;
     -Ixy,  Iyy, -Iyz;
     -Ixz, -Iyz,  Izz ];

invI = inv(I);

%% 大气参数
rho0 = 1.225;         % kg/m^3
a0 = 340.294;         % m/s

%% CL(α) 插值表 (来源: §7, 9节点)
% 使用 pchip 插值，不用多项式拟合
alpha_CL_table = [ -3.150;  0.000;  0.139;  0.200;  0.260;  0.290;  0.320;  0.500;  3.150 ];  % rad
CL_table       = [  0.000;  0.138;  1.320;  1.480;  1.760;  1.750;  1.600;  1.500;  0.000 ];

%% Cm(α) 插值表 (来源: §8, 14节点)
% ⚠️ 符号依赖MSFS坐标系，导入后必须验证 ∂Cm/∂α 的实际稳定性符号
alpha_Cm_table = [ -3.150; -0.800; -0.400; -0.200; -0.100;  0.000;  0.200;  0.230;  0.260;  0.290;  0.310;  0.400;  0.800;  3.150 ];  % rad
Cm_table       = [  0.000; -2.402; -1.861; -0.842; -0.442;  0.000;  1.173;  1.337;  1.489;  1.723;  1.919;  2.276;  2.992;  0.000 ];

%% 升降舵效能倍率 K_δe(α) 插值表 (来源: §9, 11节点)
% 仅用于俯仰力矩 Cm 公式！CL 公式中不乘 K_δe
% Cm 中: Cmde_eff = Cmde * K_δe(α)
alpha_Kde_table = [ -3.141593; -0.698132; -0.349066; -0.174533; -0.087266;  0.000000;  0.087266;  0.174533;  0.349066;  0.698132;  3.141593 ];  % rad
Kde_table       = [ -1.000;     0.050;     0.455;     0.853;     1.007;     1.000;     0.839;     0.693;     0.381;    -0.080;   -1.000 ];

%% 后缘襟翼 TE 插值表 (来源: §13.1, 6节点)
% 襟翼偏角 → 升力比例 g_L, 阻力比例 g_D
delta_flap_TE_tab = [ 0.000000;  0.087266;  0.174533;  0.261799;  0.349066;  0.698132 ];  % rad
gL_flap_TE_tab    = [ 0.000;     0.010;     1.300;     1.300;     1.170;     1.000 ];
gD_flap_TE_tab    = [ 0.000;     0.300;     0.630;     0.850;     0.970;     0.939 ];
delta_flap_TE_max = 0.698132;  % rad, CONF FULL

%% 前缘缝翼 LE 插值表 (来源: §13.2, 6节点)
% 缝翼偏角 → 升力比例 g_L, 阻力比例 g_D
delta_slat_LE_tab = [ 0.000000;  0.314159;  0.314334;  0.383972;  0.384147;  0.471239 ];  % rad
gL_slat_LE_tab    = [ 1.000;     1.000;     1.000;     1.000;     1.000;     1.000 ];
gD_slat_LE_tab    = [ 1.000;     0.330;     0.630;     0.850;     0.970;     0.939 ];
delta_slat_LE_max = 0.471239;  % rad, CONF FULL

%% 构型增量总系数 (来源: §6, §13, §14.3)
Delta_CL_flap_tot = 1.867;    % 襟翼增升总系数（全放时 g_L=1 对应的最大值）
Delta_CD_flap_tot = 0.1316;   % 襟翼增阻总系数
Delta_Cm_flap_tot = -0.084;   % 襟翼俯仰力矩总增量
CL_D0_clean       = 0.175;    % 光洁构型最小阻力对应 CL (§6)
CL_D0_flap        = 0.420;    % 放襟翼最小阻力对应 CL (§6)
CL_D0 = CL_D0_clean;          % 兼容旧变量名，默认光洁

%% 升力修正系数 (来源: §7, §11.1)
K_CL_cruise = 0.93;    % 巡航升力标量 (来源: cruise_lift_scalar)
K_GE        = 1.0;     % 地效倍率, 默认1.0, 近地时从 §11.1 表查 (来源: §11.1)

%% 升力系数 (来源: §10, §6)
CL0  = 0.138;          % α=0 时的升力系数 (来源: §7 表)
CLde = -1.652;         % 升降舵升力影响 (来源: §10)
CLq  = -57.116;        % 俯仰率 q̂→升力 (来源: §10)

%% 阻力系数 (来源: §6, §4.1)
CD0      = 0.01865;    % 零升阻力系数 (来源: §6)
k_i_eff  = 0.07447685; % 有效诱导阻力因子 (来源: §4.1)
K = k_i_eff;            % 兼容旧代码中的 K

%% 俯仰力矩系数 (来源: §9, §10)
Cm0  = 0.0;            % α=0 时的俯仰力矩 (来源: §8 表)
Cmde = -11.78;         % 升降舵俯仰效能基数 (来源: §9)
Cmq  = -1245.917;      % 俯仰阻尼 q̂→Cm (来源: §10)
% Cm 中实际舵效 = Cmde * K_δe(α), K_δe 仅用于 Cm, 不用于 CL

%% 侧向力系数 (来源: §10)
CSb  = -3.252;         % 侧滑侧力 (CYβ)
CYp  =  1.833;         % 滚转率 p̂→侧力 (CYp), 新增
CSr  =  17.395;        % 偏航率侧力 (CYr)
CSdr = -2.793;         % 方向舵侧力 (CYδr)

%% 滚转力矩系数 (来源: §10)
Clb  =  0.554;         % 上反效应 (Clβ)
Clp  = -2.078;         % 滚转阻尼 (Clp)
Clr  = -2.621;         % 偏航率滚转耦合 (Clr)
Clda = -0.291;         % 副翼滚转效能 (Clδa)
Cldr =  0.476;         % 方向舵滚转耦合 (Clδr)

%% 偏航力矩系数 (来源: §10)
Cnb  =  1.296;         % 风标稳定 (Cnβ)
Cnp  =  0.742;         % 滚转率偏航耦合 (Cnp)
Cnr  = -67.303;        % 偏航阻尼 (Cnr)
Cnda = -0.007;         % 副翼不利偏航 (Cnδa)
Cndr =  1.321;         % 方向舵偏航效能 (Cnδr)

%% 扰流板气动 (来源: §17)
% 对称扰流板比例: η_sym = (δ_spL + δ_spR) / (2 * δ_sp_max)
Delta_CL_sp = -0.466875; % 全扰流板卸升量
Delta_CD_sp =  0.05775;  % 全扰流板增阻量
Delta_Cm_sp =  0.023;    % 全扰流板俯仰力矩
spoiler_max_air = 0.698132; % rad, 空中扰流板最大偏角 (§5)
spoiler_max_gnd = 0.872665; % rad, 地面扰流板最大偏角 (§5)

%% 起落架气动 (来源: §6, §18)
% η_gear ∈ [0,1]: 0=收起 1=放下
Delta_CD_gear = 0.0372;  % 起落架放下增阻
Delta_Cm_gear = 0.0022;  % 起落架放下俯仰力矩

%% 襟翼增量 (来源: §6)
% 旧名兼容: Delta_CL_flap = Delta_CL_flap_tot, 等
Delta_CL_flap = Delta_CL_flap_tot;
Delta_CD_flap = Delta_CD_flap_tot;
Delta_Cm_flap = Delta_Cm_flap_tot;

%% §14.3 标定缩放量 (初值=1.0, 待配平后逐一标定)
% 纵向
s_malpha = 1.0;  s_mq    = 1.0;  s_mdeltae = 1.0;
s_Lq     = 1.0;  s_Ldeltae = 1.0;
% 横侧向
s_Ybeta = 1.0;  s_Yp = 1.0;  s_Yr = 1.0;  s_Ydeltar = 1.0;
s_lbeta = 1.0;  s_lp = 1.0;  s_lr = 1.0;  s_ldeltaa = 1.0;  s_ldeltar = 1.0;
s_nbeta = 1.0;  s_np = 1.0;  s_nr = 1.0;  s_ndeltaa = 1.0;  s_ndeltar = 1.0;

%% 气动力限制
CL_min  = -0.55;
CL_max  = max(CL_table);  % 从 CL 表取峰值 ≈1.76
CL_hard_max = CL_max;

CD_min = CD0;
CD_max = 0.80;

CS_max = 0.30;

Cm_max = 2.0;
Cl_max = 0.30;
Cn_max = 0.30;

alpha_min       = -8  * DH;
alpha_linear_max = 12 * DH;
alpha_stall     = 15 * DH;
alpha_hard      = 18 * DH;

beta_linear_max = 8  * DH;
beta_hard_max   = 15 * DH;

%% 舵面限位 (来源: §5)
de_max = 0.436332;     % rad, 升降舵最大上偏 (=25°)
da_max = 0.436332;     % rad, 副翼最大偏角 (=25°)
dr_max = 0.436332;     % rad, 方向舵最大偏角 (=25°)

de_min = -0.296706;    % rad, 升降舵最大下偏 (=-17°)
da_min = -da_max;
dr_min = -dr_max;

%% 推力 (来源: A320 CFM56 典型双发总推力)
thrust_min = 0.0;           % N
thrust_max = 240000.0;      % N, 双发总推力典型值

throttle_min = 0.0;
throttle_max = 1.0;

%% 起落架参数 — A320 几何 + 线性弹簧阻尼模型
ground_z = 0.0;       % m, 跑道地面Z坐标，前-右-下中Z向下

wheel_radius = [0.381000, 0.584208, 0.584208];  % m, 前轮/左主/右主 (来源: §14.2)
max_steer = 75 * DH;  % rad, 前轮最大转角 (来源: §28, 手轮通道)
steer_tau = 0.08;     % s, 前轮转向一阶时间常数

% 起落架相对质心的位置，单位 m (来源: §14.2)
% 前-右-下坐标系：
% x 前为正，y 右为正，z 下为正
r_gear_b = [ 11.234928,  -1.405128,  -1.405128;
              0.000000,  -4.267200,   4.267200;
              2.709672,   2.709672,   2.709672 ];

r_nose_b  = r_gear_b(:,1);
r_left_b  = r_gear_b(:,2);
r_right_b = r_gear_b(:,3);

% 弹簧刚度，单位 N/m
% 按质量比缩放: k_new = k_old * (42500/600) = k_old * 70.83
k_gear = [7.083e6, 2.125e7, 2.125e7];

% 阻尼系数，单位 N/(m/s)
% 按 sqrt(质量比) 缩放: c_new = c_old * 8.42
c_gear = [6.734e4, 1.179e5, 1.179e5];

% 地面摩擦系数
mu_gear = [0.65, 0.70, 0.70];

% 刹车摩擦系数，前轮不刹车，主轮刹车
mu_brake = [0.0, 0.55, 0.55];

% 滚动阻力系数
c_roll = [0.020, 0.018, 0.018];

% 侧向轮胎力系数，单位 N/(m/s)
% 按质量比缩放
c_lat = [9.917e5, 1.852e6, 1.852e6];

% 平滑符号函数速度参数
v_eps = 0.20;         % m/s

% 接地判断阈值
WOW_on  = 0.000;      % m
WOW_off = 0.000;      % m

% 地面法向，前-右-下中竖直方向为Z轴
e_z = [0; 0; 1];
P_g = eye(3) - e_z*e_z.';

%% 起落架静态压缩与初始重心位置
static_deflection = m * g / sum(k_gear);  % m

% 前-右-下：Z向下
%Zg0 = ground_z - wheel_radius(1) - r_gear_b(3,1) + static_deflection;
Zg0= -10000;
%% 风扰动模型参数(有色噪声)
Vwind_base_x = 0;       % 前向基准风 (m/s)
Vwind_base_y = 5;       % 侧向基准风 (m/s)
Vwind_base_z = 0;       % 垂直基准风 (m/s)

tau_wind = 10;          % 风相关时间常数 (s)
sigma_wind_x = 0.8;     % X轴风扰动标准差 (m/s)
sigma_wind_y = 3.0;     % Y轴风扰动标准差 (m/s)
sigma_wind_z = 0.5;     % Z轴风扰动标准差 (m/s)

K_wind_x = sigma_wind_x * sqrt(2 * tau_wind);
K_wind_y = sigma_wind_y * sqrt(2 * tau_wind);
K_wind_z = sigma_wind_z * sqrt(2 * tau_wind);

%% 参考速度 (来源: §12)
V_stall_clean = 87.970000;     % m/s, 光洁构型失速速度
V_stall_full  = 59.161111;     % m/s, 全襟翼失速速度
VR_min        = 61.733333;     % m/s, 最小抬轮速度
V_climb       = 118.836667;    % m/s, 爬升速度参考
V_cruise      = 234.072222;    % m/s, 巡航速度参考
VFE           = 140.973276;    % m/s, 最大放襟翼速度
VLE           = 144.044444;    % m/s, 最大放起落架速度
VNE           = 257.222222;    % m/s, 最大指示空速

%% 配平参考值
D0 = 1400;           % N,  配平阻力（待重算）
L0 = m * g;          % N,  配平升力 (=mg)
delta_p0 = -4.5;     % deg, 配平升降舵偏角（待重算）

% 爬升性能
gamma_climbmax = asin((thrust_max - D0) / (m * g)) * HD;  % 最大爬升角 (deg)
VR = VR_min;             % m/s, 抬轮速度
V_flare = 1.4 * V_stall_full;  % m/s, 拉平速度 ≈82.8

%% 保存所有插值表到独立 mat 文件 (供 Simulink 查表模块加载)
save("aero_tables.mat", ...
    "alpha_CL_table", "CL_table", ...
    "alpha_Cm_table", "Cm_table", ...
    "alpha_Kde_table", "Kde_table", ...
    "delta_flap_TE_tab", "gL_flap_TE_tab", "gD_flap_TE_tab", ...
    "delta_slat_LE_tab", "gL_slat_LE_tab", "gD_slat_LE_tab");

save("basedata.mat");
