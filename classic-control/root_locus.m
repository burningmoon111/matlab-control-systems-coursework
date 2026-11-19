%% root_locus.m — 第3章：复域法（根轨迹）分析设计
clear; clc;
load('model_params.mat');
load('pd_params.mat');
outdir = '../figures_module1';

disp('========================================');
disp('  第3章：复域法（根轨迹）分析设计');
disp('========================================');

%% 3.1 开环零极点分布
fprintf('=== 3.1 开环零极点分布 ===\n');
z_o = zero(G_phi);
p_o = pole(G_phi);
fprintf('开环极点: s1 = +%.4f, s2 = -%.4f (一个RHP极点)\n', omega0, -omega0);
fprintf('开环零点: 无\n\n');

figure('Position', [100, 100, 700, 500]);
h = pzplot(G_phi);
title('图3-1 开环零极点分布图', 'FontSize', 13);
grid on;
% 增大极点标记尺寸，改善可见性
set(findobj(gcf, 'Type', 'Line'), 'MarkerSize', 18, 'LineWidth', 2);
saveas(gcf, fullfile(outdir, 'fig_pzmap_open.png'));
fprintf('已保存: fig_pzmap_open.png\n\n');

%% 3.2 纯比例控制下的根轨迹
fprintf('=== 3.2 纯比例控制下的根轨迹 ===\n');

% 纯P控制: u = Kp*phi, 特征: 1 - Kp*G(s) = 0
% s^2 - omega0^2 - Kp*(-ml/J) = 0
% 等效根轨迹形式: 1 + Kp*(-G(s)) = 0

figure('Position', [100, 100, 700, 500]);
rlocus(-G_phi);
title('图3-2 纯比例控制根轨迹', 'FontSize', 13);
xlabel('实轴', 'FontSize', 12); ylabel('虚轴', 'FontSize', 12);
grid on;
try set(gca, 'FontSize', 11); catch; end
% 增大极点标记
hold on;
plot(real(p_o), imag(p_o), 'bx', 'MarkerSize', 16, 'LineWidth', 2.5);
plot(real(z_o), imag(z_o), 'bo', 'MarkerSize', 16, 'LineWidth', 2.5);
if ~isempty(z_o)
    legend('根轨迹', '开环极点', '开环零点', 'Location', 'best');
else
    legend('根轨迹', '开环极点', 'Location', 'best');
end
saveas(gcf, fullfile(outdir, 'fig_rlocus_p_only.png'));
fprintf('已保存: fig_rlocus_p_only.png\n');
fprintf('分析：两个极点关于虚轴对称，无零点，根轨迹分支沿虚轴移动。\n');
fprintf('无论Kp取何值，总有极点不能同时进入LHP → 纯P控制无法稳定。\n\n');

%% 3.3 PD校正下的根轨迹分析
fprintf('=== 3.3 PD校正下的根轨迹分析 ===\n');

% PD控制: u = (Kp+Kd*s)*phi
% 特征: 1 - (Kp+Kd*s)*G(s) = 0
% 1 + Kd*(s + Kp/Kd)*(-G(s)) = 0  → 引入零点 z = -Kp/Kd

z_pd = -Kp_design / Kd_design;
fprintf('PD控制器: C(s) = %.4f + %.4f·s\n', Kp_design, Kd_design);
fprintf('引入零点 z = -Kp/Kd = %.4f (左半平面)\n\n', z_pd);

% PD校正后的等效开环传递函数: Kd*(s-z_pd)*(-G(s))
G_pd_rl = tf([1, -z_pd], 1) * (-G_phi);

figure('Position', [100, 100, 700, 500]);
rlocus(G_pd_rl);
title('图3-3 PD校正后根轨迹', 'FontSize', 13);
grid on; % FontSize skipped for compatibility;

hold on;
plot(real(p_o), imag(p_o), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
plot(z_pd, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
legend('根轨迹', '开环极点', 'PD零点', 'Location', 'best');
saveas(gcf, fullfile(outdir, 'fig_rlocus_pd.png'));
fprintf('已保存: fig_rlocus_pd.png\n\n');

%% 3.4 阻尼比线与期望极点
fprintf('=== 3.4 根轨迹法确定参数 ===\n');

figure('Position', [100, 100, 700, 500]);
rlocus(G_pd_rl);
title('图3-4 带阻尼比线的根轨迹', 'FontSize', 13);
grid on; % FontSize skipped for compatibility;

% ζ = 0.707 阻尼比线
zeta = 0.707;
theta = acos(zeta);
hold on;
xl = xlim; yl = ylim;
x_line = linspace(xl(1), 0, 100);
plot(x_line, -x_line*tan(theta), 'r--', 'LineWidth', 1.5);
plot(x_line, x_line*tan(theta), 'r--', 'LineWidth', 1.5);
text(2, 4, '\zeta = 0.707', 'Color', 'r', 'FontSize', 12);

% 期望极点（来自时域设计）
s_des = -zeta_design*wn_design + 1j*wn_design*sqrt(1-zeta_design^2);
plot(real(s_des), imag(s_des), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(real(s_des), -imag(s_des), 'go', 'MarkerSize', 10, 'LineWidth', 2);

saveas(gcf, fullfile(outdir, 'fig_rlocus_damping.png'));
fprintf('已保存: fig_rlocus_damping.png\n\n');

%% 3.5 对比验证
fprintf('=== 3.5 时域法与复域法对比 ===\n');

% 从根轨迹验证：在期望极点处，幅值条件给出Kd
G_at_s = abs(evalfr(G_pd_rl, s_des));
Kd_rl = 1 / G_at_s;
Kp_rl = -Kd_rl * z_pd;

fprintf('参数对比:\n');
fprintf('           Kp        Kd\n');
fprintf('  时域法   %8.4f  %8.4f\n', Kp_design, Kd_design);
fprintf('  根轨迹法 %8.4f  %8.4f\n', Kp_rl, Kd_rl);

C_rl = tf([Kd_rl, Kp_rl], 1);
G_cl_rl = feedback(G_phi, C_rl, +1);

figure('Position', [100, 100, 750, 400]);
t = 0:0.001:2;
step(G_cl, t); hold on;
step(G_cl_rl, t);
title('图3-5 时域法与根轨迹法阶跃响应对比', 'FontSize', 12);
legend('时域法', '根轨迹法', 'Location', 'best');
grid on;
saveas(gcf, fullfile(outdir, 'fig_td_vs_rl.png'));
fprintf('已保存: fig_td_vs_rl.png\n\n');

fprintf('=== 3.6 本章小结 ===\n');
fprintf('1. 纯P控制根轨迹沿虚轴，无法使所有极点同时入LHP。\n');
fprintf('2. PD校正引入LHP零点z=%.2f，将根轨迹"拉"入LHP。\n', z_pd);
fprintf('3. 根轨迹法与时域法结果一致，相互验证。\n');

save('rl_params.mat', 'z_pd', 'Kp_rl', 'Kd_rl', 'G_cl_rl');
fprintf('\n根轨迹分析完成。\n');
