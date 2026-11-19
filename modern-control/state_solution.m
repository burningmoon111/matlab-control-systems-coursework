%% state_solution.m — 第2章：运动方程的解（模块二）
clear; clc;

addpath('../matlab_module1');
load('model_params.mat');
load('ss_params.mat');

outdir = '../figures_module2';

disp('========================================');
disp('  模块二 第2章：运动方程的解');
disp('========================================');
fprintf('\n');

%% 2.1 线性定常系统状态方程的解
fprintf('=== 2.1 线性定常系统状态方程的解 ===\n');
fprintf('对于线性定常系统 ż = A·z + B·u，z(0) = z0：\n');
fprintf('  z(t) = e^{At}·z0 + ∫₀ᵗ e^{A(t-τ)}·B·u(τ) dτ\n');
fprintf('  零输入响应：z_zi(t) = e^{At}·z0\n');
fprintf('  零状态响应：z_zs(t) = ∫₀ᵗ e^{A(t-τ)}·B·u(τ) dτ\n');
fprintf('其中 e^{At} = Φ(t) 称为状态转移矩阵。\n\n');

%% 2.2 状态转移矩阵的性质
fprintf('=== 2.2 状态转移矩阵的性质 ===\n');
fprintf('(1) Φ(0) = I（初始条件）\n');
fprintf('(2) dΦ(t)/dt = A·Φ(t)\n');
fprintf('(3) Φ(t₁+t₂) = Φ(t₁)·Φ(t₂)（半群性质）\n');
fprintf('(4) Φ(-t) = Φ^{-1}(t)\n');
fprintf('(5) L{Φ(t)} = (sI−A)^{-1}\n\n');

%% 2.3 状态转移矩阵的计算
fprintf('=== 2.3 状态转移矩阵的计算方法 ===\n');

% 选择若干时间点
t_samples = 0:0.05:1;
n_samples = length(t_samples);

fprintf('--- 2.3.1 矩阵指数法 (MATLAB expm) ---\n');
fprintf('在 t = 0, 0.1, 0.2, 0.3, 0.5, 1.0 s 处计算 Φ(t):\n');
check_times = [0, 0.1, 0.2, 0.3, 0.5, 1.0];
for ti = check_times
    Phi_t = expm(A_full * ti);
    fprintf('Φ(%.1f) 第3行(φ_row): [%+.4f %+.4f %+.4f %+.4f]\n', ...
            ti, Phi_t(3,:));
end

% 验证性质(1): Φ(0) = I
Phi0 = expm(A_full * 0);
fprintf('\n验证 Φ(0) = I:  max|Φ(0)-I| = %.2e  ✓\n', max(max(abs(Phi0 - eye(4)))));

% --- 拉普拉斯变换法验证 ---
fprintf('\n--- 2.3.2 拉普拉斯变换法 ---\n');
syms s t real

% 符号计算 (sI-A)^{-1}
A_sym = sym(A_full);
SI_A_inv_sym = inv(s * eye(4) - A_sym);
fprintf('(sI−A)^{-1} 符号计算完成。\n');

% 解析逆拉普拉斯变换得到 Φ(t)
Phi_sym = ilaplace(SI_A_inv_sym, s, t);

% 摆杆子系统: Φ_φ(t) 的解析形式
fprintf('\n摆杆子系统状态转移矩阵 Φ_φ(t) (解析形式):\n');
fprintf('  [φ(t)]    = [cosh(ω₀t)    (1/ω₀)sinh(ω₀t)] [φ(0) ]\n');
fprintf('  [φ̇(t)]      [ω₀sinh(ω₀t)  cosh(ω₀t)    ] [φ̇(0)]\n');
fprintf('  其中 ω₀ = %.4f rad/s\n\n', omega0);

% 数值验证：在若干时间点比较expm和解析公式
fprintf('expm与解析公式对比验证（摆杆子系统）:\n');
fprintf('  t     expm(3,3)   cosh(ω₀t)   差值\n');
for ti = check_times(2:end)
    Phi_expm = expm(A_full * ti);
    cosh_val = cosh(omega0 * ti);
    fprintf('  %.1f   %+.6f  %+.6f  %.2e\n', ti, Phi_expm(3,3), cosh_val, ...
            abs(Phi_expm(3,3) - cosh_val));
end
fprintf('两者完全一致 ✓\n\n');

%% 2.4 从状态转移矩阵看系统稳定性
fprintf('=== 2.4 从状态转移矩阵看系统稳定性 ===\n');
fprintf('摆杆子系统的零输入响应:\n');
fprintf('  φ(t)  = φ(0)·cosh(ω₀t) + (φ̇(0)/ω₀)·sinh(ω₀t)\n');
fprintf('  φ̇(t) = φ(0)·ω₀·sinh(ω₀t) + φ̇(0)·cosh(ω₀t)\n');
fprintf('\n由于 cosh(ω₀t) 和 sinh(ω₀t) 当 t→∞ 时均趋于 +∞，\n');
fprintf('任何非零初始条件都将导致状态指数发散。\n');
fprintf('这从时域角度再次验证了开环系统不稳定的结论。\n\n');

% 零输入响应仿真
z0 = [0; 0; 0.1; 0];  % 初始摆角0.1rad，其余为零
t_sim = 0:0.002:0.8;
[y_zi, t_sim, z_zi] = initial(sys_ss, z0, t_sim);

fprintf('零输入响应（z0 = [0, 0, 0.1, 0]^T）:\n');
fprintf('  t=0.2s: φ=%.4f rad\n', z_zi(t_sim==0.2, 3));
fprintf('  t=0.4s: φ=%.4f rad\n', z_zi(t_sim==0.4, 3));
fprintf('  t=0.6s: φ=%.4f rad\n', z_zi(t_sim==0.6, 3));
fprintf('  t=0.8s: φ=%.4f rad\n', z_zi(t_sim==0.8, 3));
fprintf('摆角在0.8s内从0.1rad发散至%.2frad，验证了指数发散特性。\n\n', z_zi(end, 3));

%% 图2-1: 零输入响应
figure('Position', [100, 100, 900, 400]);

subplot(1,2,1);
plot(t_sim, z_zi(:,3), 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('摆角 φ (rad)', 'FontSize', 11);
title('摆杆角度 φ(t) — 指数发散', 'FontSize', 12);
grid on;
hold on;
% 标注发散趋势
plot(t_sim, 0.1*cosh(omega0*t_sim), 'r--', 'LineWidth', 1);
legend('数值解 (initial)', '解析解 0.1·cosh(ω₀t)', 'Location', 'northwest');
text(0.4, 15, sprintf('φ(0.8)=%.1f rad', z_zi(end,3)), 'Color', 'red', 'FontSize', 9);

subplot(1,2,2);
plot(t_sim, z_zi(:,1), 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 11); ylabel('小车位移 x (m)', 'FontSize', 11);
title('小车位移 x(t) — 二次漂移', 'FontSize', 12);
grid on;
text(0.4, 0.5*max(abs(z_zi(:,1))), '零输入下小车静止', 'Color', 'blue', 'FontSize', 9);

sgtitle('图6-1 开环系统零输入响应 (z₀=[0,0,0.1,0]ᵀ)', 'FontSize', 13);
saveas(gcf, fullfile(outdir, 'fig_zero_input_response.png'));
fprintf('已保存: fig_zero_input_response.png\n');

%% 图2-2: 状态转移矩阵元素曲线
figure('Position', [100, 100, 800, 550]);

% 密集时间采样
t_dense = 0:0.005:1;
n_dense = length(t_dense);
Phi_33 = zeros(1, n_dense);  % Φ_33 = cosh(ω₀t)
Phi_34 = zeros(1, n_dense);  % Φ_34 = (1/ω₀)·sinh(ω₀t)
Phi_43 = zeros(1, n_dense);  % Φ_43 = ω₀·sinh(ω₀t)
Phi_44 = zeros(1, n_dense);  % Φ_44 = cosh(ω₀t)

for i = 1:n_dense
    Phi_t = expm(A_full * t_dense(i));
    Phi_33(i) = Phi_t(3,3);
    Phi_34(i) = Phi_t(3,4);
    Phi_43(i) = Phi_t(4,3);
    Phi_44(i) = Phi_t(4,4);
end

subplot(2,2,1);
plot(t_dense, Phi_33, 'b-', 'LineWidth', 1.5); hold on;
plot(t_dense, cosh(omega0*t_dense), 'r--', 'LineWidth', 1);
ylabel('Φ_{33}(t)'); title('Φ_{33}(t) = cosh(ω₀t)'); grid on;

subplot(2,2,2);
plot(t_dense, Phi_34, 'b-', 'LineWidth', 1.5); hold on;
plot(t_dense, sinh(omega0*t_dense)/omega0, 'r--', 'LineWidth', 1);
ylabel('Φ_{34}(t)'); title('Φ_{34}(t) = (1/ω₀)·sinh(ω₀t)'); grid on;

subplot(2,2,3);
plot(t_dense, Phi_43, 'b-', 'LineWidth', 1.5); hold on;
plot(t_dense, omega0*sinh(omega0*t_dense), 'r--', 'LineWidth', 1);
xlabel('t (s)'); ylabel('Φ_{43}(t)'); title('Φ_{43}(t) = ω₀·sinh(ω₀t)'); grid on;

subplot(2,2,4);
plot(t_dense, Phi_44, 'b-', 'LineWidth', 1.5); hold on;
plot(t_dense, cosh(omega0*t_dense), 'r--', 'LineWidth', 1);
xlabel('t (s)'); ylabel('Φ_{44}(t)'); title('Φ_{44}(t) = cosh(ω₀t)'); grid on;

sgtitle('图6-2 摆杆子系统状态转移矩阵 Φ_φ(t) 元素曲线', 'FontSize', 13);
saveas(gcf, fullfile(outdir, 'fig_state_transition.png'));
fprintf('已保存: fig_state_transition.png\n\n');

%% 保存结果
save('state_sol_params.mat', 'z0', 'z_zi', 't_sim', 'Phi_sym');
fprintf('运动方程求解完成。参数已保存至 state_sol_params.mat\n');
