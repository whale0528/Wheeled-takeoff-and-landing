%% UAV landing trajectory design
% Segment order:
% constant altitude -> steep glide -> circular flare -> shallow glide
% -> exponential flare -> touchdown.
%
% Note:
% X_aim is treated as the virtual ground-intersection point of the shallow
% glide line. The real touchdown point after exponential flare is X_touch.


%% ===== Aircraft parameters =====
g = 9.80665;
m = 600;
S_ref = 1.2;
CD0 = 0.045;
K_aero = 0.12;
rho = 1.225;
CL0 = 0.083;
CLa = 3.80;
T_max = 3000;

% CLmax estimate: CL0 + CLa * alpha_stall(13 deg) ~= 0.95
CLmax = 0.95;
Vs = sqrt(2*m*g/(rho*S_ref*CLmax));

V_app = 1.4 * Vs;       % steep glide / circular flare entry speed
V_shallow = 1.2 * Vs;   % shallow glide speed
V_flare = V_shallow;    % exponential flare speed used for X(t)

%% ===== Glide angles from paper formulas =====
T_idle = 0.05 * T_max;
qc_app = 0.5 * rho * V_app^2;
CL_app = m*g / (qc_app * S_ref);
CD_app = CD0 + K_aero * CL_app^2;
D_app = qc_app * S_ref * CD_app;

gamma1_arg = (T_idle - D_app) / (m*g);
gamma1 = asin(max(min(gamma1_arg, 1), -1));

Hdot_shallow = 2.0;      % target shallow-glide sink rate, m/s
use_paper_gamma2 = true; % false keeps the old model value, -1.5 deg
gamma2_model_deg = -1.5;

if use_paper_gamma2
    gamma2_arg = -Hdot_shallow / V_shallow;
    gamma2 = asin(max(min(gamma2_arg, 1), -1));
else
    gamma2 = gamma2_model_deg * pi/180;
end

a1 = abs(gamma1);
a2 = abs(gamma2);

%% ===== Geometry design parameters =====
X_zero = -3000;       % steep-glide ground intersection, m
R = 8000;             % circular flare radius, m
X_aim = -500;         % shallow-glide virtual ground intersection, m
H_exp = 20;           % exponential flare start height, m
sigma_design = 6;     % nominal exponential flare time constant, s
H_entry = 1000;       % landing profile entry height, m
H_touch = 0.1;        % numerical touchdown height for the exponential, m
dt = 0.05;

%% ===== Tangent circular arc geometry =====
X_A = (X_zero*tan(a1) + X_aim*tan(a2)) / (tan(a1) + tan(a2));
H_A = -(X_A - X_zero) * tan(a1);

M = [tan(a1), 1; tan(a2), 1];
bv = [tan(a1)*X_zero + R/cos(a1);
      tan(a2)*X_aim  + R/cos(a2)];
sol = M \ bv;
X_R = sol(1);
H_R = sol(2);

X_B = X_R - R * sin(a1);
H_B = H_R - R * cos(a1); 
X_C = X_R - R * sin(a2);
H_C = H_R - R * cos(a2);

X_D = X_aim - H_exp / tan(a2);
H_D = H_exp;
X_entry = X_zero - H_entry / tan(a1);

arc_length = R * (a1 - a2);
H_arc_drop = R * (cos(a2) - cos(a1));

%% ===== Exponential flare endpoint =====
sigma = sigma_design;
t_flare_end = sigma * log(H_exp / H_touch);
X_touch = X_D + V_flare * t_flare_end;

% If X_aim is forced to be the touchdown point, this is the required sigma.
t_to_Xaim = (X_aim - X_D) / V_flare;
sigma_for_Xaim = t_to_Xaim / log(H_exp / H_touch);
H_at_Xaim_with_design_sigma = H_exp * exp(-t_to_Xaim / sigma_design);

%% ===== Print design result =====
fprintf('========== Aircraft and speed ==========\n');
fprintf('Vs        = %.3f m/s\n', Vs);
fprintf('V_app     = %.3f m/s\n', V_app);
fprintf('V_shallow = %.3f m/s\n', V_shallow);
fprintf('V_flare   = %.3f m/s\n\n', V_flare);

fprintf('========== Glide angles ==========\n');
fprintf('gamma1 = %.4f deg\n', gamma1*180/pi);
fprintf('gamma2 = %.4f deg', gamma2*180/pi);
if use_paper_gamma2
    fprintf('  (asin(-Hdot_shallow/V_shallow))\n');
else
    fprintf('  (fixed model value)\n');
end
fprintf('\n');

fprintf('========== Geometry ==========\n');
fprintf('A line intersection: X = %.3f, H = %.3f\n', X_A, H_A);
fprintf('Entry: X = %.3f, H = %.3f\n', X_entry, H_entry);
fprintf('B circular entry: X = %.3f, H = %.3f\n', X_B, H_B);
fprintf('C circular exit : X = %.3f, H = %.3f\n', X_C, H_C);
fprintf('D flare start   : X = %.3f, H = %.3f\n', X_D, H_D);
fprintf('Circular center : X_R = %.3f, H_R = %.3f, R = %.1f\n', X_R, H_R, R);
fprintf('Arc length = %.3f m, arc height drop = %.3f m\n\n', ...
    arc_length, H_arc_drop);

fprintf('========== Exponential flare ==========\n');
fprintf('X_aim is the shallow-glide virtual ground point: %.3f m\n', X_aim);
fprintf('With sigma = %.3f s, H_touch = %.3f m -> X_touch = %.3f m\n', ...
    sigma, H_touch, X_touch);
fprintf('If X_aim must be touchdown, required sigma = %.3f s\n', sigma_for_Xaim);
fprintf('With sigma = %.3f s, height at X_aim would be %.3f m\n\n', ...
    sigma_design, H_at_Xaim_with_design_sigma);

%% ===== Generate profile arrays =====
% 1) Steep glide
X2 = segment_grid(X_entry, X_B, V_app*dt);
H2 = -(X2 - X_zero) * tan(a1);
gamma2_cmd = gamma1 * ones(size(X2));
V2 = V_app * ones(size(X2));

% 2) Circular flare
n_arc = max(100, ceil(arc_length/(V_app*dt)) + 1);
ang = linspace(-a1, -a2, n_arc)';
X3 = X_R + R * sin(ang);
H3 = H_R - R * cos(ang);
gamma3_cmd = ang;
V3 = linspace(V_app, V_shallow, n_arc)';

% 3) Shallow glide
X4 = segment_grid(X_C, X_D, V_shallow*dt);
H4 = (X_aim - X4) * tan(a2);
gamma4_cmd = gamma2 * ones(size(X4));
V4 = V_shallow * ones(size(X4));

% 4) Exponential flare
t_flare = (0:dt:t_flare_end)';
if t_flare(end) < t_flare_end
    t_flare = [t_flare; t_flare_end];
end
X5 = X_D + V_flare * t_flare;
H5 = H_exp * exp(-t_flare/sigma);
Hdot5 = -H5 / sigma;
gamma5_cmd = asin(max(min(Hdot5 ./ V_flare, 1), -1));
V5 = V_flare * ones(size(X5));

% Avoid duplicate boundary samples when concatenating.
X = [X2; X3(2:end); X4(2:end); X5(2:end)];
H = [H2; H3(2:end); H4(2:end); H5(2:end)];
gamma_cmd = [gamma2_cmd; gamma3_cmd(2:end); gamma4_cmd(2:end); gamma5_cmd(2:end)];
V_cmd = [V2; V3(2:end); V4(2:end); V5(2:end)];

%% ===== Plot =====
figure('Position', [50 50 1400 650], 'Color', 'w');

subplot(2, 3, [1 2 4 5]);
hold on; grid on; box on;
fill([0 3000 3000 0], [-5 -5 0 0], [0.75 0.75 0.75], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.5);
text(1500, -2.5, 'runway 3000 m', 'HorizontalAlignment', 'center', ...
    'FontSize', 9, 'Color', [0.4 0.4 0.4]);
plot([min(X)-300, max(X)+300], [0 0], 'k-', 'LineWidth', 1.2);
plot(X2, H2, 'b', 'LineWidth', 2.2);
plot(X3, H3, 'm', 'LineWidth', 2.2);
plot(X4, H4, 'c', 'LineWidth', 2.2);
plot(X5, H5, 'Color', [0.2 0.8 0.2], 'LineWidth', 2.2);
plot([X_B X_C X_D X_aim X_touch], [H_B H_C H_D 0 H_touch], ...
    'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w');
text(X_B+40, H_B+15, 'B');
text(X_C+40, H_C+15, 'C');
text(X_D+40, H_D+15, 'D');
text(X_aim+40, 15, 'X_{aim}');
text(X_touch+40, 15, 'X_{touch}');
xlabel('X (m)');
ylabel('H (m)');
title('Landing trajectory');
xlim([X_entry-300, max(X_touch, X_aim)+400]);
ylim([-5, H_entry+50]);
legend({'runway', 'steep glide', 'circular flare', 'shallow glide', ...
    'exponential flare'}, 'Location', 'southwest', 'FontSize', 9);

subplot(2, 3, 3);
hold on; grid on; box on;
plot(X3, H3, 'm', 'LineWidth', 3);
plot(X_B, H_B, 'b.', 'MarkerSize', 15);
plot(X_C, H_C, 'c.', 'MarkerSize', 15);
title(sprintf('Circular flare: dH = %.1f m, arc = %.0f m', ...
    H_arc_drop, arc_length));
xlabel('X (m)');
ylabel('H (m)');
pad = max(100, arc_length * 0.3);
xlim([X_B-pad, X_C+pad]);
ylim([H_C-H_arc_drop*3, H_B+H_arc_drop*3]);

subplot(2, 3, 6);
yyaxis left;
plot(X, gamma_cmd*180/pi, 'LineWidth', 1.3);
ylabel('\gamma_{cmd} (deg)');
yyaxis right;
plot(X, V_cmd, 'LineWidth', 1.3);
ylabel('V_{cmd} (m/s)');
grid on; box on;
xlabel('X (m)');
title('Guidance commands');

sgtitle(sprintf(['X_{zero}=%.0f m, R=%.0f m, gamma1=%.2f deg, ' ...
    'gamma2=%.2f deg, sigma=%.1f s'], ...
    X_zero, R, gamma1*180/pi, gamma2*180/pi, sigma));

%% ===== Simulink command formulas =====
fprintf('========== Simulink guidance formulas ==========\n');
fprintf('Phase 8 steep glide:\n');
fprintf('  H_cmd = (X - %.3f) * tan(%.8f)\n', X_zero, gamma1);
fprintf('  gamma_cmd = %.8f rad, V_cmd = %.3f m/s\n', gamma1, V_app);
fprintf('Phase 9 circular flare:\n');
fprintf('  H_cmd = %.3f - sqrt(max(%.3f^2 - (X - %.3f)^2, 0))\n', ...
    H_R, R, X_R);
fprintf('  gamma_cmd = atan((X - %.3f) / sqrt(max(%.3f^2 - (X - %.3f)^2, eps)))\n', ...
    X_R, R, X_R);
fprintf('  V_cmd should transition %.3f -> %.3f m/s\n', V_app, V_shallow);
fprintf('Phase 10 shallow glide:\n');
fprintf('  H_cmd = (%.3f - X) * tan(%.8f)\n', X_aim, a2);
fprintf('  gamma_cmd = %.8f rad, V_cmd = %.3f m/s\n', gamma2, V_shallow);
fprintf('Phase 11 exponential flare:\n');
fprintf('  t_flare = max((X - %.3f) / %.3f, 0)\n', X_D, V_flare);
fprintf('  H_cmd = %.3f * exp(-t_flare / %.3f)\n', H_exp, sigma);
fprintf('  gamma_cmd = asin((-H_cmd / %.3f) / %.3f)\n', sigma, V_flare);

save('landing_trajectory.mat', ...
    'g', 'm', 'S_ref', 'CD0', 'K_aero', 'rho', 'CL0', 'CLa', 'CLmax', ...
    'Vs', 'V_app', 'V_shallow', 'V_flare', ...
    'gamma1', 'gamma2', 'Hdot_shallow', ...
    'X_zero', 'R', 'X_aim', 'H_exp', 'sigma', 'sigma_design', ...
    'H_entry', 'H_touch', ...
    'X_entry', 'X_A', 'H_A', 'X_B', 'H_B', 'X_C', 'H_C', ...
    'X_D', 'H_D', 'X_R', 'H_R', 'X_touch', ...
    'arc_length', 'H_arc_drop', ...
    'X', 'H', 'gamma_cmd', 'V_cmd');

function Xseg = segment_grid(x0, x1, dx)
    n = max(2, ceil(abs(x1 - x0) / abs(dx)) + 1);
    Xseg = linspace(x0, x1, n)';
end
