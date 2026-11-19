%% main_run.m — 模块一：经典控制部分 主脚本
% 依次运行：系统建模 → 时域分析 → 根轨迹分析 → 频域分析

clear; clc;

% 确保图像输出目录存在
outdir = '../figures_module1';
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

fprintf('==================================================\n');
fprintf('   模块一：直线一级倒立摆经典控制部分\n');
fprintf('   GIP-100-L 型直线一级倒立摆\n');
fprintf('==================================================\n\n');

try
    fprintf('>>> 第1章：系统建模...\n');
    run('sys_modeling.m');

    fprintf('\n>>> 第2章：时域法分析设计...\n');
    run('time_domain.m');

    fprintf('\n>>> 第3章：复域法（根轨迹）分析设计...\n');
    run('root_locus.m');

    fprintf('\n>>> 第4章：频域法分析设计...\n');
    run('freq_domain.m');

    fprintf('\n==================================================\n');
    fprintf('   模块一全部完成！\n');
    fprintf('   图像输出目录: figures_module1/\n');
    fprintf('==================================================\n');

catch ME
    fprintf(2, '\n错误: %s\n', ME.message);
    fprintf(2, '发生在: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
end
