


%% 无人机着陆轨迹
%  定高平飞→陡下滑→圆弧拉平→浅下滑→指数拉飘→接地
%  从 X_zero 出发推导全部几何


%% ===== 飞机参数 (plannew.m) =====
g=9.80665; m=600; S_ref=1.2; CD0=0.045; K_aero=0.12; rho=1.225;
CL0=0.083; CLa=3.80; T_max=3000;
% CLmax 推算: CL0 + CLa * alpha_stall(13deg) = 0.95
CLmax = 0.95;
Vs = sqrt(2*m*g/(rho*S_ref*CLmax));
V_app = 1.4 * Vs;

%% ===== 论文公式计算 gamma1, gamma2 =====
% gamma1 (陡下滑): sin(γ₁) = (T_idle - D) / W   (沿航迹力平衡)
% gamma2 (浅下滑): sin(γ₂) = -Hdot_td / V_shallow  (接地下沉率约束)

% 陡下滑段: V=V_app, 油门怠速 5%
T_idle = 0.05 * T_max;
qc_app = 0.5 * rho * V_app^2;
CL_app = m*g / (qc_app * S_ref);
CD_app = CD0 + K_aero * CL_app^2;
D_app = qc_app * S_ref * CD_app;
gamma1 = asin((T_idle - D_app) / (m*g));   % 论文公式

% 浅下滑段: V=1.2*Vs, 论文公式 gamma2 = asin(-Hdot_td / V_shallow)
% 接地下沉率取 2.0 m/s (本机速度高, 0.5m/s 会导致 gamma2 过浅轨迹不可行)
Hdot_td = 2.0;                              % 接地下沉率 m/s
V_shallow = 1.2 * Vs;
gamma2 = -1.5/57.3      ;  % 论文公式

fprintf('=== 论文公式计算结果 ===\n');
fprintf('  gamma1: sin(γ)=(%.0f-%.0f)/%.0f=%.4f  → γ₁=%.2f°\n', ...
    T_idle, D_app, m*g, sin(gamma1), gamma1*180/pi);
fprintf('  gamma2: sin(γ)=%.1f/%.1f=%.4f  → γ₂=%.2f°\n\n', ...
    -Hdot_td, V_shallow, sin(gamma2), gamma2*180/pi);

%% ===== 轨迹几何设计参数 =====
X_zero  = -3000;           % 陡下滑线地面交点 X (m)
R       = 8000;            % 圆弧半径 (m)
X_aim   = -500;            % 接地点 X (m)
H_exp   = 20;              % 拉飘起始高度 (m)
sigma   = 6;               % 拉飘时间常数 (s)
H_entry = 1001;             % 进场高度 (m)

a1 = abs(gamma1);  a2 = abs(gamma2);

%% ===== 几何推导 =====
% A: 两下滑线交点
X_A = (X_zero*tan(a1) + X_aim*tan(a2)) / (tan(a1) + tan(a2));
H_A = -(X_A - X_zero) * tan(a1);

% 切角 θ = (180° - γ₁ + γ₂)/2
theta = (pi - a1 + a2) / 2;

% 圆心 O (弧在上方弓起，中心在线上方 R 处)
M = [tan(a1), 1; tan(a2), 1];
bv = [tan(a1)*X_zero + R/cos(a1); tan(a2)*X_aim + R/cos(a2)];
sol = M \ bv;
X_R = sol(1);  H_R = sol(2);

% 切点 B(陡侧,左)、C(浅侧,右)
X_B = X_R - R * sin(a1);   H_B = H_R - R * cos(a1);
X_C = X_R - R * sin(a2);   H_C = H_R - R * cos(a2);

% D 点 (拉飘起始) 和 entry
X_D = X_aim - H_exp / tan(a2);
X_entry = X_zero - H_entry / tan(a1);

arc_length = R * (a1 - a2);         % 弧段水平长
H_arc_drop = R * (cos(a2)-cos(a1));  % 弧段下降量

fprintf('========== X_zero = %.0fm ==========\n\n', X_zero);
fprintf('各点坐标:\n');
fprintf('  A(交点)   X=%8.0f  H=%7.1f\n', X_A, H_A);
fprintf('  B(陡→弧) X=%8.0f  H=%7.1f\n', X_B, H_B);
fprintf('  C(弧→浅) X=%8.0f  H=%7.1f\n', X_C, H_C);
fprintf('  D(拉飘)   X=%8.0f  H=%7.1f\n', X_D, H_exp);
fprintf('  接地      X=%8.0f  H=  0.0\n', X_aim);
fprintf('\n各段:\n');
fprintf('  ①定高  H=%.0f  (X<%.0f)\n', H_entry, X_entry);
fprintf('  ②陡下滑 %.0f→%.0fm  %.0fm  %.0f°\n', H_entry, H_B, X_B-X_entry, a1*180/pi);
fprintf('  ③圆弧   %.1f→%.1fm %4.0fm  R=%.0f  ΔH=%.1f\n', H_B, H_C, arc_length, R, H_arc_drop);
fprintf('  ④浅下滑 %.0f→%.0fm  %.0fm  %.0f°\n', H_C, H_exp, X_D-X_C, a2*180/pi);
fprintf('  ⑤拉飘   %.0f→0m  %.0fm  σ=%.0fs\n', H_exp, X_aim-X_D, sigma);
fprintf('  总: %.0fm\n', X_aim-X_entry);

%% ===== 轨迹时间序列 =====
dt = 0.05;

% ② 陡下滑
X2 = (X_entry : V_app*dt : X_B)';  H2 = -(X2 - X_zero) * tan(a1);

% ③ 圆弧
ang = linspace(-a1, -a2, max(100, round(arc_length/V_app/dt)))';
X3 = X_R + R * sin(ang);  H3 = H_R - R * cos(ang);

% ④ 浅下滑
X4 = (X_C : V_app*dt : X_D)';  H4 = (X_aim - X4) * tan(a2);

% ⑤ 拉飘: H(t) = H_exp·e^(-t/σ) + Hdot_td·σ·(1-e^(-t/σ))
Hdot_td = 0.3;  % 安全接地垂直速度
t_flare = (0 : dt : sigma*log(H_exp/0.1))';
H5 = H_exp * exp(-t_flare/sigma);
X5 = X_D + V_app * t_flare;

X = [X2; X3; X4; X5];  H = [H2; H3; H4; H5];

%% ===== 可视化 =====
figure('Position',[50 50 1400 600],'Color','w');

subplot(2,3,[1 2 4 5]); hold on; grid on; box on;
fill([0 3000 3000 0],[-5 -5 0 0],[0.75 0.75 0.75],'EdgeColor','none','FaceAlpha',0.5);
text(1500,-2.5,'跑道 3000m','HorizontalAlignment','center','FontSize',9,'Color',[0.4 0.4 0.4]);
plot([X_entry-200 X_aim+200],[0 0],'k-','LineWidth',1.5);

plot(X2,H2,'b','LineWidth',2.2);
plot(X3,H3,'m','LineWidth',2.2);
plot(X4,H4,'c','LineWidth',2.2);
plot(X5,H5,'Color',[0.2 0.8 0.2],'LineWidth',2.2);

plot(X_B, H_B, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w');
plot(X_C, H_C, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w');
plot(X_D, H_exp, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w');
plot(X_aim, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w');
text(X_B+40, H_B+15, 'B');
text(X_C+40, H_C+15, 'C');
text(X_D+40, H_exp+15, 'D');
text(X_aim+40, 15, '接地');
% 弧放大框
rect_h = H_arc_drop + 35;
rectangle('Position',[X_C-80 H_C-15 arc_length+160 rect_h],...
    'EdgeColor',[0.5 0.5 0.5],'LineStyle',':','LineWidth',1);

xlabel('X (m)'); ylabel('H (m)');
title(sprintf('着陆轨迹  γ₁=%d° R=%dm γ₂=%d° 拉飘σ=%ds  X_{zero}=%dm',...
    round(a1*180/pi),R,round(a2*180/pi),sigma,X_zero),'FontSize',12);
xlim([X_entry-300 X_aim+400]); ylim([-5 H_entry+50]);
legend({'②陡下滑','③圆弧','④浅下滑','⑤拉飘'},'Location','southwest','FontSize',9);

% 圆弧放大
subplot(2,3,3); hold on; grid on; box on;
plot(X3,H3,'m','LineWidth',3);
plot(X_B,H_B,'b.','MarkerSize',15);
plot(X_C,H_C,'c.','MarkerSize',15);
title(sprintf('圆弧放大  ΔH=%.1fm 弧长%.0fm',H_arc_drop,arc_length),'FontSize',9);
xlabel('X (m)'); ylabel('H (m)');
pad = arc_length * 0.3;
xlim([X_B-pad, X_C+pad]);
ylim([H_C-H_arc_drop*3, H_B+H_arc_drop*3]);

% 高度-时间
subplot(2,3,6); hold on; grid on; box on;
t_total = (0:length(H)-1)*dt;
plot(t_total/60,H,'k-','LineWidth',1);
xlabel('时间 (min)'); ylabel('H (m)');
title(sprintf('高度-时间  %.1fmin',t_total(end)/60),'FontSize',10);

sgtitle(sprintf('X_{zero}=%dm  γ₁=%d°  R=%dm  γ₂=%d°  H_{exp}=%dm  σ=%ds  V_{app}=%.0fm/s',...
    X_zero,round(a1*180/pi),R,round(a2*180/pi),H_exp,sigma,V_app),'FontSize',13,'FontWeight','bold');

%% ===== Simulink H_cmd =====
fprintf('\n========== Simulink: H_cmd = f(X_current) ==========\n');
fprintf('if    X <= %.0f   →  ① H_cmd = %.0f\n', X_entry, H_entry);
fprintf('elseif X <= %.0f  →  ② H_cmd = -(X - %.0f) * tan(%.0f°)\n', X_B, X_zero, a1*180/pi);
fprintf('elseif X <= %.0f  →  ③ H_cmd = %.1f - sqrt(%.0f^2 - (X - %.0f)^2)\n', X_C, H_R, R, X_R);
fprintf('elseif X <= %.0f  →  ④ H_cmd = -(X - %.0f) * tan(%.0f°)\n', X_D, X_aim, a2*180/pi);
fprintf('elseif H > 0.5    →  ⑤ H_cmd = %.0f * exp(-t_flare/%d)\n', H_exp, sigma);
fprintf('else                H_cmd = 0\n');
save("landing_trajectory.mat");