%% 起落架总体配置
basedata;
ground_z = 0.0;                 % m, 跑道地面Z坐标
static_pitch_gear = -0.003491;  % rad, 地面静止俯仰角
static_cg_height = 2.709672;    % m, 地面静止CG高度

VLE = 144.044444;               % m/s, 最大放起落架速度 (=280 kt)
max_gear_extended = VLE;        % m/s, 与源文件命名保持一致

Delta_CD_gear = 0.0372;         % 起落架放下增阻
Delta_Cm_gear = 0.0022;         % 起落架放下俯仰力矩增量

%% 起落架接地点与轮心几何
% 源文件CONTACT_POINTS给出的是接地点；当前Simulink压缩量模块使用轮心加轮胎半径，
% 因此 r_gear_b 保持为轮心坐标，真实接地点另存为 r_gear_contact_b。
wheel_radius = [0.381000, 0.584210, 0.584210];  % m, 前轮/左主/右主

r_gear_contact_b = [ 11.234928,  -1.405128,  -1.405128;
                      0.000000,  -4.267200,   4.267200;
                      2.910840,   2.996184,   2.996184 ];

r_gear_center_b = r_gear_contact_b;
r_gear_center_b(3,:) = r_gear_contact_b(3,:) - wheel_radius;

r_gear_b = r_gear_center_b;

r_nose_b  = r_gear_b(:,1);
r_left_b  = r_gear_b(:,2);
r_right_b = r_gear_b(:,3);

%% 起落架压缩量、阻尼比和弹簧指数
delta_static = [0.304800, 0.376428, 0.376428];  % m, 静压缩量
delta_gear_max    = [0.573024, 0.523235, 0.523235];  % m, 最大压缩量
compress_ratio = delta_gear_max ./ delta_static;

damping_ratio = [1.05, 0.45, 0.45];             % 阻尼比
spring_exp    = [2.05, 1.05, 1.05];             % 弹簧指数

gear_extend_time  = [10.6, 11.1, 11.1];         % s, 放下时间
gear_retract_time = [ 9.4,  9.9,  9.9];         % s, 收起时间

%% 载荷分配与等效线性弹簧阻尼
% 载荷比例由空机CG纵向位置与前/主起落架纵向位置按力矩平衡得到。
x_cg_ft   = -9.42;
x_nose_ft = 27.44;
x_main_ft = -14.03;

nose_load_frac = (x_cg_ft - x_main_ft) / (x_nose_ft - x_main_ft);
main_load_frac = (1 - nose_load_frac) / 2;
gear_load_frac = [nose_load_frac, main_load_frac, main_load_frac];

W_empty_gear = 416785.020685;   % N, 93697 lbf
W_mtow_gear  = 774724.517622;   % N, 174165 lbf

F_static_empty = W_empty_gear * gear_load_frac;
m_eff_empty = F_static_empty / g;
k_gear_empty = F_static_empty ./ delta_static;
c_gear_empty = 2 .* damping_ratio .* sqrt(k_gear_empty .* m_eff_empty);

F_static_mtow = W_mtow_gear * gear_load_frac;
m_eff_mtow = F_static_mtow / g;
k_gear_mtow = F_static_mtow ./ delta_static;
c_gear_mtow = 2 .* damping_ratio .* sqrt(k_gear_mtow .* m_eff_mtow);

if ~exist("landing_gear_ref_mass_mode", "var")
    landing_gear_ref_mass_mode = "current";  % "current" / "empty" / "mtow"
end

landing_gear_ref_mass_mode = lower(char(landing_gear_ref_mass_mode));

switch landing_gear_ref_mass_mode
    case "empty"
        W_gear_ref = W_empty_gear;
    case "mtow"
        W_gear_ref = W_mtow_gear;
    otherwise
        W_gear_ref = m * g;
end

F_static_gear = W_gear_ref * gear_load_frac;
m_eff_gear = F_static_gear / g;

k_gear = F_static_gear ./ delta_static;                 % N/m
c_gear = 2 .* damping_ratio .* sqrt(k_gear .* m_eff_gear);  % N/(m/s)

K_gear = F_static_gear ./ (delta_static .^ spring_exp); % N/m^p, 非线性弹簧系数
K_gear_mtow = F_static_mtow ./ (delta_static .^ spring_exp);

%% 转向参数
max_steer = 75 * DH;              % rad, A320前轮最大可控转角
max_steer_contact_cfg = 95 * DH;  % rad, 源CONTACT_POINTS前轮最大转角字段
steer_tau = 0.08;                 % s, 前轮转向一阶时间常数

max_speed_full_steering = 91.440000;       % m/s, 300 ft/s
max_speed_decreasing_steering = 106.680000;% m/s, 350 ft/s


%% 摩擦、滚阻与刹车参数
% A320源文件没有给出物理轮胎-跑道mu，以下为当前简化模型使用的跑道参数。
mu_gear = [0.65, 0.70, 0.70];     % 地面摩擦上限，前轮/左主/右主
mu_brake = [0.0, 0.55, 0.55];     % 刹车摩擦系数，前轮不刹车
c_roll = [0.020, 0.018, 0.018];   % 滚动阻力系数

V_lat_ref = 0.20;                 % m/s, 侧向力平滑参考速度
mu_lat = mu_gear;
c_lat = mu_lat .* F_static_gear / V_lat_ref;  % N/(m/s), 按法向载荷缩放

v_eps = 0.20;                     % m/s, 平滑符号函数速度参数

toe_brakes_scale = 0.518;         % 脚蹬刹车缩放
brake_map = [0, 1, 2];            % 0无刹车，1左主轮，2右主轮

P_brake_norm_max = 17498894.010060;          % Pa, 2538 psi
P_brake_park_no_manual = 14499674.587532;    % Pa, 2103 psi
P_brake_altn_antiskid_off = 7997918.460075;  % Pa, 1160 psi
P_brake_altn_max = P_brake_norm_max;         % Pa

%% 接地判定与地面投影
WOW_on  = 0.005;                  % m, 接地判定阈值
WOW_off = 0.002;                  % m, 离地判定阈值，需模型加入滞环后使用

e_z = [0; 0; 1];                  % 地面法向，Z向下
P_g = eye(3) - e_z*e_z.';

%% 起落架收放状态
gear_downlocked = [1, 1, 1];      % 前轮/左主/右主锁定放下
eta_gear = 1.0;                   % 气动模型用：0收起，1放下

%% 地面初始位置参考
static_deflection_gear = delta_static;
static_deflection = delta_static(1);  % m, 兼容旧Zg0公式时使用前轮静压缩量
Zg0_gear_static = ground_z + static_deflection - r_gear_b(3,1) - wheel_radius(1);
Zg0=-10000;