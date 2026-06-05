clc;
clear; 

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

%% 飞机基本参数
m = 600.0;            % kg
g = 9.80665;          % m/s^2

S_ref = 1.2;          % m^2
b_ref = 2.0;          % m
c_ref = 0.9;          % m

%% 转动惯量
Ixx = 120.0;          % kg*m^2, 绕 X 轴滚转惯量
Iyy = 480.0;          % kg*m^2, 绕 Y 轴偏航惯量
Izz = 520.0;          % kg*m^2, 绕 Z 轴俯仰惯量

Ixy = 0.0;
Ixz = 0.0;
Iyz = 0.0;

I = [ Ixx, -Ixy, -Ixz;
     -Ixy,  Iyy, -Iyz;
     -Ixz, -Iyz,  Izz ];

invI = inv(I);

%% 大气参数
rho0 = 1.225;         % kg/m^3
a0 = 340.294;         % m/s

%% 升力
CL0  = 0.083;
CLa  = 3.80;
CLq  = 4.50;
CLde = 0.28;

%% 阻力
CD0 = 0.045;
K   = 0.12;

%% 俯仰力矩
Cm0  = 0.060;
Cma  = -0.80;
Cmq  = -18.0;
Cmde = -0.90;     % 正升降舵 -> 负俯仰力矩

%% 侧向力
CSb  = -0.70;
CSr  = 0.12;
CSdr = 0.18;

%% 滚转力矩
Clb  = -0.08;
Clp  = -0.70;
Clr  = 0.08;
Clda = -0.20;     % 如果正副翼也定义为产生负滚转力矩

%% 偏航力矩
Cnb  = 0.25;
Cnp  = -0.04;
Cnr  = -0.45;
Cndr = -0.15;     % 正方向舵 -> 负偏航力矩
%% 舵面与推力限制
de_max = 25;     % d, 升降舵最大偏角
da_max = 25;     % d, 副翼最大偏角
dr_max = 25;     % d, 方向舵最大偏角

de_min = -de_max;
da_min = -da_max;
dr_min = -dr_max;

thrust_min = 0.0;     % N
thrust_max = 3000.0;  % N

throttle_min = 0.0;
throttle_max = 1.0;

%% 起落架参数
ground_z = 0.0;       % m, 跑道地面Z坐标，前-右-下中Z向下

wheel_radius = 0.18;  % m
max_steer = 25 * DH;  % rad, 前轮最大转角
steer_tau = 0.08;     % s, 前轮转向一阶时间常数

% 起落架相对质心的位置，单位 m
% 前-右-下坐标系：
% x 前为正，y 右为正，z 下为正
% 列顺序：前轮、左主轮、右主轮
r_gear_b = [ 2.00,    -1/3,   -1/3;
             0.00,    -0.80,   0.80;
             0.65,     0.65,   0.65 ];

r_nose_b  = r_gear_b(:,1);
r_left_b  = r_gear_b(:,2);
r_right_b = r_gear_b(:,3);

% 弹簧刚度，单位 N/m
k_gear = [1.0e5, 3.0e5, 3.0e5];

% 阻尼系数，单位 N/(m/s)
c_gear = [0.8e4, 1.4e4, 1.4e4];

% 地面摩擦系数
mu_gear = [0.65, 0.70, 0.70];

% 刹车摩擦系数，前轮不刹车，主轮刹车
mu_brake = [0.0, 0.55, 0.55];

% 滚动阻力系数
c_roll = [0.020, 0.018, 0.018];

% 侧向轮胎力系数，单位 N/(m/s)
c_lat = [1.4e4, 2.2e4, 2.2e4];

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
% 轮心刚接触地面时：Z_wheel = ground_z - wheel_radius
% 静态压缩delta后：Z_wheel = ground_z - wheel_radius + delta
% Z_wheel = Zg0 + r_gear_b(3,1)
%Zg0 = ground_z - wheel_radius - r_gear_b(3,1) + static_deflection;
Zg0=-1000;
%% 飞机起飞初始值
Xg0 = -12000.0;            % m, 前向位置

% 如果原来的 Zg0 = 2.0 是侧偏距，现在应放到 Yg0
Yg0 = 0.0;            % m, 右向位置，向右为正
% 如果希望从跑道中心线起飞，用：
% Yg0 = 0.0;

u0_b = 128.0;           % m/s, 机体系前向速度
v0_b = 0.0;           % m/s, 机体系右向速度
w0_b = 0.0;           % m/s, 机体系下向速度

V_min = 3;            % 防止除0

phi0   = 0.0;         % rad, 滚转角，绕X轴
theta0 = 3/57.3;         % rad, 俯仰角，绕Y轴
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
%VR = 40.0;                  % m/s, 抬轮速度
H_safe = 10.0;              % m, 安全爬升高度
Hdot_safe = 1.0;            % m/s, 安全爬升率
H_climb_complete = 1000.0;  % m, 爬升完成高度

rotation_rate_cmd = 3.0 * DH;  % rad/s, 抬轮俯仰角速度指令
climb_eta_cmd = 6.0 * DH;      % rad, 初始爬升航迹倾角


% 配平时，升力和阻力
D0=1400;
L0=5102;
delta_p0=-4.5;%D

% 爬升性能
gamma_climbmax=asin((3000-D0)/(m*g))*57.3;%最大爬升角
VR=100;

save("landing_plan.mat");
control_design;
landing_trajectory;