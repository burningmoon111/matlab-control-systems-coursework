%% time_domain.m — 第2章：时域法分析设计
% GIP-100-L 直线一级倒立摆 PD控制器设计
% 控制输入：小车加速度 u = ẍ

clear; clc;
load('model_params.mat');
outdir = '../figures_module1';

disp('========================================');
disp('  第2章：时域法分析设计');
disp('========================================');
fprintf('\n');

%% 2.1 开环系统时域响应分析
fprintf('=== 2.1 开环系统时域响应分析 ===\n');

% 2.1.1 开环传递函数特征
[num_phi, den_phi] = tfdata(G_phi, 'v');
fprintf('开环传递函数 G(s) = Phi(s)/U(s):\n');
fprintf('  G(s) = %.4f / (s^2 - %.4f)\n', num_phi(3), -den_phi(3));
fprintf('  极点: s = +/-%.4f\n', omega0);
fprintf('  一个极点在右半平面(RHP) → 系统开环不稳定\n\n');

% 开环单位脉冲响应
figure('Position', [100, 100, 800, 400]);
t_ol = 0:0.001:0.4;
[y_imp, t_ol] = impulse(G_phi, t_ol);

plot(t_ol, y_imp, 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 12);
ylabel('摆角 \phi (rad)', 'FontSize', 12);
title('图2-1 开环单位脉冲响应', 'FontSize', 13);
grid on;
try set(gca, 'FontSize', 11); catch; end;
text(0.15, max(y_imp)*0.7, '\leftarrow 响应指数发散（RHP极点）', ...
     'Color', 'red', 'FontSize', 11, 'FontWeight', 'bold');
saveas(gcf, fullfile(outdir, 'fig_openloop_impulse.png'));
fprintf('已保存: fig_openloop_impulse.png\n\n');

%% 2.2 开环系统稳定性判定
fprintf('=== 2.2 开环系统稳定性判定 ===\n');
fprintf('特征方程: s^2 - %.4f = 0\n', omega0^2);
fprintf('极点: s1 = +%.4f (RHP, 不稳定), s2 = -%.4f (LHP, 稳定)\n', omega0, omega0);
fprintf('结论：系统开环不稳定，必须引入反馈控制。\n\n');

%% 2.3 PD控制器设计
fprintf('=== 2.3 PD控制器设计 ===\n');

% 2.3.1 为什么纯P控制不行
fprintf('--- 2.3.1 纯P控制分析 ---\n');

% 控制律: u = Kp*phi + Kd*phi_dot （镇定问题的正反馈形式）
% 物理意义：摆杆向右倒(phi>0)，小车需向右加速(u>0)才能接住它
%
% 闭环特征方程: den(s) - num(s)*(Kp+Kd*s) = 0
% (J*s^2 - mgl) - (-ml)*(Kp+Kd*s) = 0
% J*s^2 + ml*Kd*s + (ml*Kp - mgl) = 0

fprintf('控制律: u = Kp·φ + Kd·φ̇\n');
fprintf('物理理由：摆杆向右倒(φ>0)时，小车需向右加速(ü>0)才能接住，\n');
fprintf('         因此需要正反馈而非负反馈。\n\n');

% 闭环特征方程系数
% s^2 + (ml/J)*Kd*s + (ml*Kp - mgl)/J = 0
a2 = 1;
a1_Kd_coef = m*l/J;  % Kd 的系数
a0_Kp_coef = m*l/J;  % Kp 的系数
a0_const = -m*g*l/J;  % 常数部分

fprintf('闭环特征方程: s^2 + (ml/J)·Kd·s + (ml·Kp - mgl)/J = 0\n');
fprintf('代入数值: s^2 + %.4f·Kd·s + (%.4f·Kp - %.4f) = 0\n\n', ...
        a1_Kd_coef, a0_Kp_coef, -a0_const);

% 纯P控制(Kd=0)
fprintf('纯P控制(Kd=0)时特征方程: s^2 + (%.4f·Kp - %.4f) = 0\n', a0_Kp_coef, -a0_const);
fprintf('由劳斯判据：\n');
fprintf('  s^2 | 1    %.4f·Kp - %.4f\n', a0_Kp_coef, -a0_const);
fprintf('  s^1 | 0    0\n');
fprintf('  s^0 | %.4f·Kp - %.4f\n', a0_Kp_coef, -a0_const);
fprintf('s^1行全为零 → 存在对称极点或纯虚极点 → 系统临界稳定/不稳定。\n');
fprintf('实际上没有阻尼项(s^1系数为0)，系统无法稳定。\n');
fprintf('必须引入微分作用(Kd>0)提供阻尼。\n\n');

%% 2.3.4 劳斯判据求稳定条件
fprintf('--- 2.3.4 劳斯判据求Kp, Kd范围 ---\n');

fprintf('闭环特征方程: s^2 + %.4f·Kd·s + (%.4f·Kp - %.4f) = 0\n', ...
        a1_Kd_coef, a0_Kp_coef, -a0_const);
fprintf('\n劳斯表:\n');
fprintf('  s^2 | 1          %.4f·Kp - %.4f\n', a0_Kp_coef, -a0_const);
fprintf('  s^1 | %.4f·Kd    0\n', a1_Kd_coef);
fprintf('  s^0 | %.4f·Kp - %.4f\n', a0_Kp_coef, -a0_const);
fprintf('\n稳定性条件（劳斯表第一列全部为正）:\n');
fprintf('  1) %.4f·Kd > 0  =>  Kd > 0\n', a1_Kd_coef);
fprintf('  2) %.4f·Kp - %.4f > 0  =>  Kp > %.2f\n', a0_Kp_coef, -a0_const, -a0_const/a0_Kp_coef);

Kp_min = -a0_const / a0_Kp_coef;
fprintf('\n结论: Kp > %.2f,  Kd > 0\n\n', Kp_min);

%% 2.4 基于期望性能指标的参数整定
fprintf('=== 2.4 基于期望性能指标的参数整定 ===\n');

% 性能指标
sigma_target = 5;       % 超调量 ≤ 5%
ts_target = 2.0;        % 调节时间 ≤ 2s

zeta_design = sqrt(log(100/sigma_target)^2 / (pi^2 + log(100/sigma_target)^2));
fprintf('由 σ ≤ %d%% → ζ ≥ %.4f,  取 ζ = 0.707\n', sigma_target, zeta_design);

zeta_design = 0.707;
wn_design = 4 / (zeta_design * ts_target);
fprintf('由 ts ≤ %.1fs (2%%准则) → ωn ≥ %.4f rad/s\n', ts_target, wn_design);

% 为确保性能，取较大的ωn
wn_design = 6.0;
fprintf('取设计值 ωn = %.1f rad/s (满足ts=4/(0.707*6)=0.94s < 2s)\n\n', wn_design);

%% 2.4.3 PD参数计算
fprintf('--- 2.4.3 PD参数计算 ---\n');

% 从标准形式: s^2 + 2ζωn·s + ωn^2
% 对比: s^2 + (ml/J)Kd·s + (ml·Kp - mgl)/J
%
% Kd = 2ζωn / (ml/J) = 2ζωn·J/(ml)
% Kp = (ωn^2·J + mgl) / (ml) = ωn^2·J/(ml) + g/l

Kd_design = 2 * zeta_design * wn_design * J / (m * l);
Kp_design = (wn_design^2 * J + m*g*l) / (m * l);

fprintf('Kd = 2ζωn·J/(ml) = 2·%.3f·%.1f·%.4f/(%.2f·%.2f) = %.4f\n', ...
        zeta_design, wn_design, J, m, l, Kd_design);
fprintf('Kp = (ωn^2·J + mgl)/(ml) = (%.1f·%.4f + %.4f)/(%.2f·%.2f) = %.4f\n', ...
        wn_design^2, J, m*g*l, m, l, Kp_design);
fprintf('\n设计参数: Kp = %.2f,  Kd = %.2f\n', Kp_design, Kd_design);

% 验证满足稳定性条件
fprintf('验证: Kp=%.2f > Kp_min=%.2f ✓,  Kd=%.2f > 0 ✓\n\n', Kp_design, Kp_min, Kd_design);

%% 2.5 系统动态性能分析
fprintf('=== 2.5 系统动态性能分析 ===\n');

% 构建闭环传递函数
% 控制律: u = Kp*phi + Kd*phi_dot
% 系统: phi = G(s)*u
% 闭环: phi = G(s)*(Kp + Kd*s)*phi => (1 - G(s)*(Kp+Kd*s)) = 0
% 闭环TF (从参考输入到输出，对于镇定问题):
% phi(s)/ref(s) = G(s) / (1 - G(s)*(Kp+Kd*s))
% 但对于镇定，我们关心从初始条件或扰动到phi的响应

% 方法：使用状态空间模型
A_pend = [0, 1; m*g*l/J, 0];  % [phi, phi_dot]^T
B_pend = [0; -m*l/J];          % 输入u

% 控制律: u = Kp*phi + Kd*phi_dot = [Kp, Kd]*[phi; phi_dot]
K_ctrl = [Kp_design, Kd_design];

% 闭环A矩阵: A_cl = A + B*K (因为u = Kx进入正向，不是负反馈)
A_cl = A_pend + B_pend * K_ctrl;

fprintf('闭环系统矩阵 A_cl:\n');
disp(A_cl);

% 闭环极点
cl_poles = eig(A_cl);
fprintf('闭环极点:\n');
for i = 1:length(cl_poles)
    fprintf('  λ%d = %+.4f%+.4fj\n', i, real(cl_poles(i)), imag(cl_poles(i)));
end

% 计算实际 ζ 和 ωn
zeta_actual = -real(cl_poles(1)) / abs(cl_poles(1));
wn_actual = abs(cl_poles(1));
fprintf('实际阻尼比 ζ = %.4f,  自然频率 ωn = %.4f rad/s\n\n', zeta_actual, wn_actual);

% 闭环传递函数（用于阶跃响应仿真）
C_pd = tf([Kd_design, Kp_design], 1);
% 正反馈: TF = G*C / (1 - G*C)
G_cl = feedback(G_phi, C_pd, +1);  % positive feedback

fprintf('闭环传递函数:\n');
G_cl

% 阶跃响应
figure('Position', [100, 100, 800, 400]);
t_sim = 0:0.001:2;
[y_step, t_sim] = step(G_cl, t_sim);

plot(t_sim, y_step, 'b-', 'LineWidth', 1.5);
xlabel('时间 t (s)', 'FontSize', 12);
ylabel('摆角 \phi (rad)', 'FontSize', 12);
title('图2-3 PD控制闭环阶跃响应', 'FontSize', 13);
grid on;
try set(gca, 'FontSize', 11); catch; end;

% 计算性能指标
info = stepinfo(y_step, t_sim, y_step(end));
sigma_actual = info.Overshoot;
t_peak = info.PeakTime;
ts_actual = info.SettlingTime;

fprintf('\n实际性能指标:\n');
fprintf('  超调量 σ = %.2f%%\n', sigma_actual);
fprintf('  峰值时间 tp = %.3f s\n', t_peak);
fprintf('  调节时间 ts = %.3f s (2%%准则)\n', ts_actual);
fprintf('  稳态值 = %.6f\n', y_step(end));

% 标注性能指标
if ~isnan(t_peak) && t_peak > 0
    hold on;
    [y_max, idx_max] = max(y_step);
    plot([t_peak, t_peak], [0, y_max], 'r--', 'LineWidth', 1);
    if ~isnan(ts_actual)
        plot([ts_actual, ts_actual], [min(y_step), max(y_step)], 'g--', 'LineWidth', 1);
        text(ts_actual+0.05, y_max*0.8, sprintf('t_s=%.2fs', ts_actual), ...
             'Color', 'g', 'FontSize', 10);
    end
    text(t_peak+0.05, y_max*0.5, sprintf('t_p=%.2fs\nσ=%.1f%%', t_peak, sigma_actual), ...
         'Color', 'r', 'FontSize', 10);
end

saveas(gcf, fullfile(outdir, 'fig_pd_step_response.png'));
fprintf('已保存: fig_pd_step_response.png\n\n');

%% 2.6 系统稳态性能分析
fprintf('=== 2.6 系统稳态性能分析 ===\n');

% 系统型别：G(s)没有原点极点 → 0型系统
% 但镇定问题不涉及参考跟踪，重点在扰动抑制

% 扰动稳态误差
% 对于输入扰动d（等效在加速度输入端），传函: G/(1-G*C)
G_dist = feedback(G_phi, C_pd, +1);
ess_dist = dcgain(G_dist);

fprintf('系统型别: 0型（传递函数无原点极点）\n');
fprintf('镇定问题中，稳态性能体现在对常值扰动的抑制能力:\n');
fprintf('  单位阶跃扰动→摆角稳态偏差 = %.6f rad\n', ess_dist);
fprintf('  等效静刚度 = %.2f (N·m/rad)\n', 1/abs(ess_dist));
fprintf('  Kp越大，静刚度越高，抗扰动能力越强。\n');

% 保存结果
save('pd_params.mat', 'Kp_design', 'Kd_design', 'zeta_design', 'wn_design', ...
     'C_pd', 'G_cl', 'A_cl', 'cl_poles', 'sigma_actual', 't_peak', 'ts_actual', ...
     'a1_Kd_coef', 'a0_Kp_coef', 'a0_const', 'Kp_min');

fprintf('\n时域分析设计完成。\n');
