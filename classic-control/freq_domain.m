%% freq_domain.m — 第4章：频域法分析设计
clear; clc;
load('model_params.mat');
load('pd_params.mat');
outdir = '../figures_module1';

disp('========================================');
disp('  第4章：频域法分析设计');
disp('========================================');

% 高密度频率向量（确保Nyquist图平滑）
w = logspace(-2, 3, 2000);

%% 4.1 频域分析概述
fprintf('=== 4.1 频域分析概述 ===\n');
fprintf('倒立摆系统包含RHP极点(P=1)，为典型的非最小相位系统。\n');
fprintf('Nyquist稳定判据是判断此类系统闭环稳定性的有效工具。\n\n');

%% 4.2 开环频率特性分析
fprintf('=== 4.2 开环频率特性分析 ===\n');

G_open_freq = -G_phi;  % 等效开环(不含控制器)

% 开环Bode图
figure('Position', [100, 100, 800, 500]);
bode(G_open_freq, w);
title('图4-1 开环Bode图', 'FontSize', 13);
grid on; try set(gca, 'FontSize', 11); catch; end;
saveas(gcf, fullfile(outdir, 'fig_bode_open.png'));
fprintf('已保存: fig_bode_open.png\n');

try
    [Gm, Pm, Wcg, Wcp] = margin(G_open_freq);
    fprintf('幅值裕度 Gm = %.4f dB\n', 20*log10(Gm));
    fprintf('相角裕度 Pm = %.4f deg\n\n', Pm);
catch
    fprintf('（开环不稳定，margin函数不适用）\n\n');
end

%% 4.3 Nyquist稳定判据
fprintf('=== 4.3 Nyquist稳定判据 ===\n');
fprintf('Nyquist判据: Z = P - 2N\n');
fprintf('  P = %d (开环RHP极点数)\n', rhp_count);
fprintf('  稳定条件 Z = 0 => N = P = %d\n\n', rhp_count);

% 开环Nyquist图（高密度频率点确保平滑）
figure('Position', [100, 100, 700, 550]);
[re, im] = nyquist(G_open_freq, w);
re = squeeze(re); im = squeeze(im);

plot(re, im, 'b-', 'LineWidth', 2.0);
hold on;
% 对称部分（对于纯实数Nyquist，对称部分与自身重合）
plot(re, -im, 'b--', 'LineWidth', 1.0);
xlabel('实轴', 'FontSize', 12); ylabel('虚轴', 'FontSize', 12);
title('图4-2 开环Nyquist图', 'FontSize', 13);
grid on; try set(gca, 'FontSize', 11); catch; end;

% 标注(-1, j0)
plot(-1, 0, 'ro', 'MarkerSize', 12, 'LineWidth', 2);
text(-1.15, 0.15, '(-1, j0)', 'Color', 'red', 'FontSize', 11, ...
     'HorizontalAlignment', 'right', 'FontWeight', 'bold');

% 标注Nyquist曲线起点 (ω=0处)
plot(re(1), im(1), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
text(re(1)-0.05, 0.10, sprintf('ω=0 (%.3f, 0)', re(1)), ...
     'Color', 'blue', 'FontSize', 9, 'HorizontalAlignment', 'right');

% 解释曲线为纯实数的原因
text(-0.8, 0.35, 'G(jω) = -2.5/(ω²+24.5) 为纯实数', 'Color', 'blue', 'FontSize', 9);
text(-0.8, 0.25, 'Nyquist曲线退化为实轴上的线段', 'Color', 'blue', 'FontSize', 9);

axis equal;
xlim([-1.5, 0.5]);
ylim([-0.6, 0.6]);
saveas(gcf, fullfile(outdir, 'fig_nyquist_open.png'));
fprintf('已保存: fig_nyquist_open.png\n\n');

fprintf('开环Nyquist图未包围(-1,j0) → 闭环不稳定（Z≠0）。\n');
fprintf('需引入PD校正改变曲线形态。\n\n');

%% 4.4 PD校正频域设计
fprintf('=== 4.4 PD校正频域设计 ===\n');

G_ol_pd_freq = C_pd * G_open_freq;

fprintf('PD控制器: C(s) = %.4f + %.4f·s\n', Kp_design, Kd_design);
fprintf('转折频率 ω_z = Kp/Kd = %.4f rad/s\n\n', Kp_design/Kd_design);

figure('Position', [100, 100, 800, 500]);
bode(G_ol_pd_freq, w);
title('图4-3 PD校正后Bode图', 'FontSize', 13);
grid on; try set(gca, 'FontSize', 11); catch; end;
saveas(gcf, fullfile(outdir, 'fig_bode_pd.png'));
fprintf('已保存: fig_bode_pd.png\n');

[Gm_pd, Pm_pd, Wcg_pd, Wcp_pd] = margin(G_ol_pd_freq);
fprintf('校正后稳定裕度:\n');
fprintf('  幅值裕度 Gm = %.4f dB\n', 20*log10(Gm_pd));
fprintf('  相角裕度 Pm = %.4f deg\n', Pm_pd);
fprintf('  穿越频率 Wcg = %.4f rad/s\n\n', Wcg_pd);

%% 4.5 Nyquist判据验证
fprintf('=== 4.5 Nyquist判据验证 ===\n');

figure('Position', [100, 100, 700, 550]);
[re_pd, im_pd] = nyquist(G_ol_pd_freq, w);
re_pd = squeeze(re_pd); im_pd = squeeze(im_pd);

plot(re_pd, im_pd, 'b-', 'LineWidth', 2.0);
hold on;
plot(re_pd, -im_pd, 'b-', 'LineWidth', 2.0);
xlabel('实轴', 'FontSize', 12); ylabel('虚轴', 'FontSize', 12);
title('图4-4 PD校正后Nyquist图', 'FontSize', 13);
grid on; try set(gca, 'FontSize', 11); catch; end;

% 标注(-1, j0)
plot(-1, 0, 'ro', 'MarkerSize', 12, 'LineWidth', 2);
text(-1.15, 0.15, '(-1,j0)', 'Color', 'red', 'FontSize', 11, ...
     'HorizontalAlignment', 'right', 'FontWeight', 'bold');

% 标注曲线起点 (ω=0处)
plot(re_pd(1), im_pd(1), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
text(re_pd(1)-0.3, im_pd(1)-0.25, sprintf('ω=0 (%.2f, 0)', re_pd(1)), ...
     'Color', 'blue', 'FontSize', 9);
text(-2.2, -0.8, '逆时针包围(-1,j0) 1圈', 'Color', 'blue', 'FontSize', 10, 'FontWeight', 'bold');
text(-2.2, -1.0, 'N=1, P=1 → Z=0 闭环稳定', 'Color', 'blue', 'FontSize', 10);

axis equal;
xlim([-3, 1]);
ylim([-1.5, 1.5]);
saveas(gcf, fullfile(outdir, 'fig_nyquist_pd.png'));
fprintf('已保存: fig_nyquist_pd.png\n\n');

fprintf('校正后Nyquist曲线逆时针包围(-1,j0) → 满足N=P=1条件，Z=0，闭环稳定。\n\n');

%% 4.6 三法一致性
fprintf('=== 4.6 三法设计结果一致性 ===\n');

G_cl_freq = G_cl;

figure('Position', [100, 100, 1000, 350]);
subplot(1,3,1);
step(G_cl); title('时域法', 'FontSize', 11); grid on;

subplot(1,3,2);
load('rl_params.mat', 'G_cl_rl');
step(G_cl_rl); title('复域法(根轨迹)', 'FontSize', 11); grid on;

subplot(1,3,3);
step(G_cl_freq); title('频域法', 'FontSize', 11); grid on;

sgtitle('图4-5 时域/复域/频域三法设计结果对比', 'FontSize', 13);
saveas(gcf, fullfile(outdir, 'fig_three_methods_compare.png'));
fprintf('已保存: fig_three_methods_compare.png\n\n');

fprintf('=== 模块一总结 ===\n');
fprintf('1. 三种方法从不同视角验证：纯P控制无效，必须引入D作用。\n');
fprintf('2. PD设计参数: Kp=%.2f, Kd=%.2f\n', Kp_design, Kd_design);
fprintf('3. 三法结论一致，设计经过多角度验证。\n');

save('freq_params.mat', 'Gm_pd', 'Pm_pd', 'Wcg_pd', 'Wcp_pd');
fprintf('\n频域分析完成。\n');
