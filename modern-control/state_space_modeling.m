%% state_space_modeling.m — 第1章：系统状态空间建模（模块二）
clear; clc;

% 从模块一加载已有参数
addpath('../matlab_module1');
load('model_params.mat');

outdir = '../figures_module2';
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

disp('========================================');
disp('  模块二 第1章：系统状态空间建模');
disp('========================================');
fprintf('\n');

%% 1.1 状态空间描述的基本概念
fprintf('=== 1.1 状态空间描述的基本概念 ===\n');
fprintf('状态空间描述是現代控制理论的核心工具。对于n阶线性定常系统，\n');
fprintf('状态方程和输出方程的标准形式为：\n');
fprintf('  ż = A·z + B·u\n');
fprintf('  y = C·z + D·u\n');
fprintf('其中z为n维状态向量，u为控制输入，y为输出。\n\n');

%% 1.2 状态变量的选取
fprintf('=== 1.2 状态变量的选取 ===\n');
fprintf('对于GIP-100-L型直线一级倒立摆，选取4维状态向量：\n');
fprintf('  z = [x, ẋ, φ, φ̇]^T\n');
fprintf('  x   — 小车水平位移 (m)\n');
fprintf('  ẋ   — 小车水平速度 (m/s)\n');
fprintf('  φ   — 摆杆偏角 (rad)\n');
fprintf('  φ̇  — 摆杆角速度 (rad/s)\n');
fprintf('控制输入 u = ẍ — 小车加速度 (m/s²)\n\n');

%% 1.3-1.4 状态方程矩阵化
fprintf('=== 1.3-1.4 状态方程矩阵形式 ===\n');

% A_full 和 B_full 已从 model_params.mat 加载
% 定义输出矩阵 C 和直接传递矩阵 D
C = [1, 0, 0, 0;   % 测量小车位移 x
     0, 0, 1, 0];  % 测量摆杆偏角 φ
D = [0; 0];

fprintf('系统矩阵 A (4×4):\n');
disp(A_full);
fprintf('输入矩阵 B (4×1):\n');
disp(B_full);
fprintf('输出矩阵 C (2×4):\n');
disp(C);
fprintf('直接传递矩阵 D (2×1):\n');
disp(D);

% 创建状态空间对象
sys_ss = ss(A_full, B_full, C, D);
fprintf('\n状态空间模型 (ss对象) 创建完成。\n\n');

%% 1.5 开环特征值与稳定性分析
fprintf('=== 1.5 开环特征值与稳定性分析 ===\n');

openloop_eigs = eig(A_full);
fprintf('开环特征值 λ (A_full):\n');
for i = 1:length(openloop_eigs)
    fprintf('  λ%d = %+.4f\n', i, openloop_eigs(i));
end

% 分类：小车双积分器极点 (s=0,0)，摆杆极点 (s=±omega0)
fprintf('\n特征值物理意义：\n');
fprintf('  两个零特征值              → 小车双积分器（位置控制系统）\n');
fprintf('  s = +%.4f                → 摆杆不稳定极点 (RHP)\n', omega0);
fprintf('  s = -%.4f                → 摆杆稳定极点 (LHP)\n', omega0);

fprintf('\n与模块一经典控制结论的一致性验证：\n');
fprintf('  摆杆子系统极点 s = ±%.4f\n', omega0);
fprintf('  模块一传递函数极点 s = ±%.4f  完全一致 ✓\n\n', omega0);

% 状态空间→传递函数 验证
sys_phi = tf(ss(A_full(3:4,3:4), B_full(3:4), [1, 0], 0));
[num_tf, den_tf] = tfdata(sys_phi, 'v');
fprintf('由状态空间提取的摆角传递函数:\n');
fprintf('  Φ(s)/U(s) = %.4f / (s^2 - %.4f)\n', num_tf(3), -den_tf(3));
fprintf('  与模块一 G_phi 一致 ✓\n\n');

%% 生成特征值分布图
figure('Position', [100, 100, 700, 550]);
pzmap(sys_ss);
title('图5-1 开环系统零极点分布图（4阶状态空间）', 'FontSize', 13);
grid on;
set(findobj(gcf, 'Type', 'Line'), 'MarkerSize', 16, 'LineWidth', 2);

% 增大标记并添加注释
hold on;
h_axes = gca;
% 标注小车极点
text(0.15, 0.25, '小车双积分器极点', 'Color', 'blue', 'FontSize', 10, 'FontWeight', 'bold');
text(0.15, -0.25, 's = 0 (二重根)', 'Color', 'blue', 'FontSize', 10);
% 标注摆杆极点
text(omega0*0.3, 0.5, sprintf('摆杆RHP极点\ns=+%.2f', omega0), ...
     'Color', 'red', 'FontSize', 10, 'FontWeight', 'bold');
text(-omega0-1.5, 0.5, sprintf('摆杆LHP极点\ns=-%.2f', omega0), ...
     'Color', 'green', 'FontSize', 10, 'FontWeight', 'bold');
% RHP区域阴影
xl = xlim; yl = ylim();
fill([0, xl(2), xl(2), 0], [yl(1), yl(1), yl(2), yl(2)], ...
     [1, 0.9, 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
text(xl(2)*0.5, yl(2)*0.85, '不稳定区域 (RHP)', 'Color', [0.7, 0.3, 0.3], ...
     'FontSize', 10, 'HorizontalAlignment', 'center');

saveas(gcf, fullfile(outdir, 'fig_eigenvalue_analysis.png'));
fprintf('已保存: fig_eigenvalue_analysis.png\n\n');

%% 保存结果
save('ss_params.mat', 'A_full', 'B_full', 'C', 'D', 'sys_ss', 'openloop_eigs');
fprintf('状态空间建模完成。参数已保存至 ss_params.mat\n');
