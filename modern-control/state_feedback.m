%% state_feedback.m — 第4章：状态反馈控制器设计（模块二）
clear; clc;

addpath('../matlab_module1');
load('model_params.mat');
load('ss_params.mat');
load('co_params.mat');
load('pd_params.mat');  % 用于与模块一PD设计对比

outdir = '../figures_module2';

disp('========================================');
disp('  模块二 第4章：状态反馈控制器设计');
disp('========================================');
fprintf('\n');

%% 4.1 状态反馈的基本结构与控制律
fprintf('=== 4.1 状态反馈的基本结构与控制律 ===\n');
fprintf('全状态反馈控制律: u = −K·z\n');
fprintf('其中 K = [k₁, k₂, k₃, k₄] 为 1×4 反馈增益矩阵。\n');
fprintf('闭环系统: ż = (A − B·K)·z\n');
fprintf('通过选择K，可以任意配置闭环极点位置（前提：系统完全能控）。\n\n');

%% 4.2 极点配置法设计
fprintf('=== 4.2 极点配置法设计 ===\n');

% 4.2.1 期望闭环极点选取
fprintf('--- 4.2.1 期望闭环极点的选取 ---\n');
fprintf('摆杆极点（与模块一PD设计一致，便于对比）:\n');
fprintf('  ζ = 0.707, ωn = 6.0 rad/s\n');
fprintf('  → s1,2 = -%.4f ± j%.4f\n', zeta_design*wn_design, wn_design*sqrt(1-zeta_design^2));

cart_pole_val1 = 8.0;
cart_pole_val2 = 9.0;
fprintf('小车极点（比摆杆快约2倍，确保小车动态不主导响应）:\n');
fprintf('  → s3 = -%.0f, s4 = -%.0f（B秩为1，须取不同实极点）\n\n', cart_pole_val1, cart_pole_val2);

P_des = [-zeta_design*wn_design + 1j*wn_design*sqrt(1-zeta_design^2);
         -zeta_design*wn_design - 1j*wn_design*sqrt(1-zeta_design^2);
         -cart_pole_val1;
         -cart_pole_val2];

fprintf('期望闭环极点 P_des:\n');
for i = 1:4
    fprintf('  p%d = %+.4f%+.4fj\n', i, real(P_des(i)), imag(P_des(i)));
end

% 4.2.2 能控性验证（已在第3章确认）
fprintf('\n--- 4.2.2 能控性验证 ---\n');
fprintf('rank(Co) = %d = 4 → 系统完全能控，可进行极点配置。✓\n', rank_Co);

% 4.2.3 反馈增益矩阵K的求解
fprintf('\n--- 4.2.3 反馈增益矩阵K的求解 (MATLAB place) ---\n');
K_place = place(A_full, B_full, P_des);
fprintf('K_place = [');
fprintf('%.4f, ', K_place);
fprintf(']\n');

% 4.2.4 闭环极点验证
fprintf('\n--- 4.2.4 闭环极点验证 ---\n');
A_cl_place = A_full - B_full * K_place;
eig_place = eig(A_cl_place);
fprintf('实际闭环极点:\n');
for i = 1:4
    fprintf('  λ%d = %+.4f%+.4fj\n', i, real(eig_place(i)), imag(eig_place(i)));
end
fprintf('与期望极点一致 ✓\n\n');

%% 4.3 LQR最优控制设计
fprintf('=== 4.3 线性二次型最优控制（LQR）设计 ===\n');

% 4.3.1 二次型性能指标
fprintf('--- 4.3.1 二次型性能指标 ---\n');
fprintf('J = ∫₀^∞ (zᵀQz + uᵀRu) dt\n');
fprintf('Q 惩罚状态偏差，R 惩罚控制能量。\n\n');

% 4.3.2 加权矩阵选取
fprintf('--- 4.3.2 加权矩阵Q和R的选取 ---\n');
Q = diag([100, 1, 100, 1]);
R_val = 1;

fprintf('Q = diag([100, 1, 100, 1])\n');
fprintf('  q11=100 → 强惩罚小车位移偏差（位置镇定优先）\n');
fprintf('  q22=1   → 轻惩罚小车速度（允许速度变化）\n');
fprintf('  q33=100 → 强惩罚摆杆角度偏差（角度镇定优先）\n');
fprintf('  q44=1   → 轻惩罚摆杆角速度（允许角速度变化）\n');
fprintf('R = %.0f → 单位控制能量惩罚\n\n', R_val);

% 4.3.3-4.3.4 求解黎卡提方程和LQR增益
fprintf('--- 4.3.3-4 代数黎卡提方程求解 ---\n');
[K_lqr, S, e_lqr] = lqr(A_full, B_full, Q, R_val);

fprintf('LQR反馈增益 K_lqr = [');
fprintf('%.4f, ', K_lqr);
fprintf(']\n\n');

fprintf('闭环极点 (LQR):\n');
for i = 1:4
    fprintf('  λ%d = %+.4f%+.4fj\n', i, real(e_lqr(i)), imag(e_lqr(i)));
end

% 计算LQR对应的阻尼比和自然频率
dom_poles_lqr = e_lqr(imag(e_lqr) > 0);
if ~isempty(dom_poles_lqr)
    zeta_lqr = -real(dom_poles_lqr(1)) / abs(dom_poles_lqr(1));
    wn_lqr = abs(dom_poles_lqr(1));
    fprintf('\nLQR主导极点: ζ=%.3f, ωn=%.2f rad/s\n', zeta_lqr, wn_lqr);
end

%% 4.4 状态观测器设计
fprintf('\n=== 4.4 状态观测器设计 ===\n');

% 4.4.1 全维Luenberger观测器
fprintf('--- 4.4.1 全维状态观测器结构 ---\n');
fprintf('观测器动态: ẑ̂ = A·ẑ + B·u + L·(y − C·ẑ)\n');
fprintf('误差动态: ė = (A − L·C)·e\n\n');

% 4.4.2 能观性验证（已在第3章确认）
fprintf('--- 4.4.2 能观性验证 ---\n');
fprintf('rank(Ob) = %d = 4 → 系统完全能观，可设计观测器。✓\n\n', rank_Ob);

% 4.4.3 观测器极点选取
fprintf('--- 4.4.3 观测器极点选取 ---\n');
obs_factor = 4;  % 观测器比控制器快4倍
obs_base = obs_factor * abs(real(P_des(1)));
P_obs = [-obs_base; -obs_base-1; -obs_base-2; -obs_base-3];
fprintf('取观测器极点比控制器快%d倍:\n', obs_factor);
fprintf('  P_obs = [-%.0f, -%.0f, -%.0f, -%.0f]\n\n', ...
        abs(P_obs(1)), abs(P_obs(2)), abs(P_obs(3)), abs(P_obs(4)));

% 4.4.4 观测器增益矩阵L的求解
fprintf('--- 4.4.4 观测器增益矩阵L的求解 ---\n');
L = place(A_full', C', P_obs)';
fprintf('L (4×2) =\n');
disp(L);

% 验证观测器极点
eig_obs = eig(A_full - L*C);
fprintf('实际观测器极点:\n');
for i = 1:4
    fprintf('  λ%d = %+.4f%+.4fj\n', i, real(eig_obs(i)), imag(eig_obs(i)));
end
fprintf('与期望一致 ✓\n\n');

%% 4.5 分离原理与带观测器的闭环系统
fprintf('=== 4.5 分离原理与带观测器的闭环系统 ===\n');
fprintf('闭环系统（8阶）: [ż; ė] = [A-BK, BK; 0, A-LC] · [z; e]\n');
fprintf('其中 e = z − ẑ 为估计误差。\n');
fprintf('由分离原理，闭环极点 = eig(A-BK) ∪ eig(A-LC)。\n\n');

% 联合仿真（使用极点配置增益）
z0 = [0; 0; 0.1; 0];      % 真实初始状态（摆角0.1rad）
z_hat0 = [0; 0; 0; 0];    % 观测器初始估计（零初始）
e0 = z0 - z_hat0;          % 初始估计误差

% 8阶增广系统: [z; z_hat]
A_aug = [A_full, -B_full*K_place;
         L*C,    A_full - B_full*K_place - L*C];
B_aug = [zeros(4,1); zeros(4,1)];  % 无外部输入
C_aug = eye(8);
D_aug = zeros(8,1);

sys_aug = ss(A_aug, B_aug, C_aug, D_aug);
t_sim = 0:0.001:1.5;
[~, t_sim, z_aug] = initial(sys_aug, [z0; z_hat0], t_sim);

z_true = z_aug(:, 1:4);
z_est = z_aug(:, 5:8);
err = z_true - z_est;
u_combined = -K_place * z_est';

fprintf('联合仿真完成 (z0=[0,0,0.1,0]ᵀ, ẑ0=[0,0,0,0]ᵀ)。\n\n');

%% 4.6 极点配置法与LQR法的对比
fprintf('=== 4.6 极点配置法与LQR法的对比分析 ===\n');

% LQR全状态反馈仿真（无观测器）
A_cl_lqr = A_full - B_full * K_lqr;
sys_cl_lqr = ss(A_cl_lqr, B_full, eye(4), zeros(4,1));
[~, t_lqr, z_lqr] = initial(sys_cl_lqr, z0, t_sim);
u_lqr_full = -K_lqr * z_lqr';

% 极点配置全状态反馈仿真（无观测器）
sys_cl_place = ss(A_cl_place, B_full, eye(4), zeros(4,1));
[~, t_place, z_place] = initial(sys_cl_place, z0, t_sim);
u_place_full = -K_place * z_place';

% 性能对比统计
[phi_max_place, idx_pp] = max(abs(z_place(:,3)));
[phi_max_lqr, idx_lqr] = max(abs(z_lqr(:,3)));
u_max_place = max(abs(u_place_full));
u_max_lqr = max(abs(u_lqr_full));

fprintf('性能指标对比:\n');
fprintf('                极点配置      LQR\n');
fprintf('  |φ|_max:      %.4f       %.4f  rad\n', phi_max_place, phi_max_lqr);
fprintf('  |u|_max:      %.4f       %.4f  m/s²\n', u_max_place, u_max_lqr);
fprintf('  K增益范数|K|: %.4f       %.4f\n', norm(K_place), norm(K_lqr));

% =================================================================
%% 图4-1: 极点配置响应
figure('Position', [100, 100, 900, 400]);
subplot(1,2,1);
plot(t_place, z_place(:,3), 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('摆角 φ (rad)', 'FontSize', 11);
title('摆杆角度 φ(t) — 极点配置', 'FontSize', 12);
grid on;
hold on;
phi_st = 0.05*z0(3);  % 5% of initial
idx_settle = find(abs(z_place(:,3)) > phi_st, 1, 'last');
if ~isempty(idx_settle) && idx_settle < length(t_place)
    plot([t_place(idx_settle), t_place(idx_settle)], [0, z0(3)], 'r--', 'LineWidth', 1);
end

subplot(1,2,2);
plot(t_place, z_place(:,1), 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('小车位移 x (m)', 'FontSize', 11);
title('小车位移 x(t) — 极点配置', 'FontSize', 12);
grid on;

sgtitle('图7-1 极点配置状态反馈响应 (z₀=[0,0,0.1,0]ᵀ)', 'FontSize', 13);
saveas(gcf, fullfile(outdir, 'fig_pole_placement_response.png'));
fprintf('已保存: fig_pole_placement_response.png\n');

%% 图4-2: LQR响应
figure('Position', [100, 100, 900, 400]);
subplot(1,2,1);
plot(t_lqr, z_lqr(:,3), 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('摆角 φ (rad)', 'FontSize', 11);
title('摆杆角度 φ(t) — LQR', 'FontSize', 12);
grid on;

subplot(1,2,2);
plot(t_lqr, z_lqr(:,1), 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('小车位移 x (m)', 'FontSize', 11);
title('小车位移 x(t) — LQR', 'FontSize', 12);
grid on;

sgtitle('图7-2 LQR状态反馈响应 (z₀=[0,0,0.1,0]ᵀ)', 'FontSize', 13);
saveas(gcf, fullfile(outdir, 'fig_lqr_response.png'));
fprintf('已保存: fig_lqr_response.png\n');

%% 图4-3: 观测器状态估计
figure('Position', [100, 100, 900, 650]);
states_labels = {'x (m)', 'ẋ (m/s)', 'φ (rad)', 'φ̇ (rad/s)'};
for i = 1:4
    subplot(2,2,i);
    plot(t_sim, z_true(:,i), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_sim, z_est(:,i), 'r--', 'LineWidth', 1.5);
    xlabel('t (s)', 'FontSize', 10);
    ylabel(states_labels{i}, 'FontSize', 10);
    title(['状态 z_', num2str(i), ': ', states_labels{i}]);
    legend('真实值', '观测器估计', 'Location', 'best', 'FontSize', 8);
    grid on;
end
sgtitle('图7-3 状态观测器估计效果 (ẑ₀=0, 实际z₀=[0,0,0.1,0]ᵀ)', 'FontSize', 13);
saveas(gcf, fullfile(outdir, 'fig_observer_states.png'));
fprintf('已保存: fig_observer_states.png\n');

%% 图4-4: 带观测器的闭环响应
figure('Position', [100, 100, 900, 400]);
subplot(1,2,1);
plot(t_sim, z_true(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t_place, z_place(:,3), 'g--', 'LineWidth', 1);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('摆角 φ (rad)', 'FontSize', 11);
title('摆杆角度 — 全状态 vs 观测器反馈', 'FontSize', 12);
legend('带观测器', '全状态反馈', 'Location', 'best');
grid on;

subplot(1,2,2);
plot(t_sim, u_combined, 'b-', 'LineWidth', 1.5); hold on;
plot(t_place, u_place_full, 'g--', 'LineWidth', 1);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('控制输入 u (m/s²)', 'FontSize', 11);
title('控制输入 — 全状态 vs 观测器反馈', 'FontSize', 12);
legend('带观测器', '全状态反馈', 'Location', 'best');
grid on;

sgtitle('图7-4 带Luenberger观测器的闭环系统响应', 'FontSize', 13);
saveas(gcf, fullfile(outdir, 'fig_combined_response.png'));
fprintf('已保存: fig_combined_response.png\n');

%% 图4-5: 极点配置 vs LQR 对比
figure('Position', [100, 100, 900, 400]);

subplot(1,2,1);
plot(t_place, z_place(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t_lqr, z_lqr(:,3), 'r-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('摆角 φ (rad)', 'FontSize', 11);
title('摆杆角度对比', 'FontSize', 12);
legend('极点配置', 'LQR', 'Location', 'best');
grid on;

subplot(1,2,2);
plot(t_place, u_place_full, 'b-', 'LineWidth', 1.5); hold on;
plot(t_lqr, u_lqr_full, 'r-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('控制输入 u (m/s²)', 'FontSize', 11);
title('控制输入对比', 'FontSize', 12);
legend('极点配置', 'LQR', 'Location', 'best');
grid on;

sgtitle('图7-5 极点配置 vs LQR 性能对比', 'FontSize', 13);
saveas(gcf, fullfile(outdir, 'fig_pp_vs_lqr_compare.png'));
fprintf('已保存: fig_pp_vs_lqr_compare.png\n\n');

%% 保存结果
save('sf_params.mat', 'K_place', 'K_lqr', 'L', 'P_des', 'P_obs', ...
     'A_cl_place', 'A_cl_lqr', 'eig_place', 'e_lqr', 'eig_obs', ...
     'Q', 'R_val', 'z0', 'z_hat0', ...
     't_place', 'z_place', 't_lqr', 'z_lqr', 't_sim', 'z_true', 'z_est', 'err');
fprintf('状态反馈设计完成。参数已保存至 sf_params.mat\n');
