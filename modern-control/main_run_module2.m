%% main_run_module2.m — 模块二：现代控制部分 主脚本
clear; clc;

addpath('../matlab_module1');

outdir = '../figures_module2';
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

fprintf('==================================================\n');
fprintf('   模块二：直线一级倒立摆现代控制部分\n');
fprintf('   GIP-100-L 型直线一级倒立摆\n');
fprintf('==================================================\n\n');

try
    fprintf('>>> 第1章：状态空间建模...\n');
    run('state_space_modeling.m');

    fprintf('\n>>> 第2章：运动方程的解...\n');
    run('state_solution.m');

    fprintf('\n>>> 第3章：能控性与能观性分析...\n');
    run('controllability_observability.m');

    fprintf('\n>>> 第4章：状态反馈控制器设计...\n');
    run('state_feedback.m');

    fprintf('\n==================================================\n');
    fprintf('   模块二全部完成！\n');
    fprintf('   图像输出目录: figures_module2/\n');
    fprintf('==================================================\n');

catch ME
    fprintf(2, '\n错误: %s\n', ME.message);
    fprintf(2, '发生在: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
end
