%% controllability_observability.m — 第3章：能控性与能观性分析（模块二）
clear; clc;

addpath('../matlab_module1');
load('model_params.mat');
load('ss_params.mat');

disp('========================================');
disp('  模块二 第3章：能控性与能观性分析');
disp('========================================');
fprintf('\n');

%% 3.1 能控性定义与基本概念
fprintf('=== 3.1 能控性定义与基本概念 ===\n');
fprintf('能控性定义：对于线性定常系统，若存在一个无约束的控制输入u(t)，\n');
fprintf('能在有限时间[t₀, t_f]内将系统从任意初始状态z(t₀)转移到任意终\n');
fprintf('端状态z(t_f)，则称系统状态完全能控。\n\n');
fprintf('物理意义：小车加速度ü能否将所有4个状态变量驱动到任意期望值？\n\n');

%% 3.2 能控性的代数秩判据
fprintf('=== 3.2 能控性的代数秩判据 ===\n');

% 构造能控性判别矩阵 Co = [B, AB, A²B, A³B]
fprintf('--- 3.2.1 能控性判别矩阵构造 ---\n');
Co = ctrb(A_full, B_full);
fprintf('Co = [B | AB | A²B | A³B] (4×4):\n');
disp(Co);

% 计算秩
fprintf('--- 3.2.2 矩阵秩计算 ---\n');
rank_Co = rank(Co);
fprintf('rank(Co) = %d\n', rank_Co);

if rank_Co == 4
    fprintf('rank(Co) = 4 = n（系统阶数）→ 系统完全能控 ✓\n');
else
    fprintf('rank(Co) = %d < 4 → 系统不完全能控 ✗\n', rank_Co);
end

% 能控性矩阵的数值特性
fprintf('\n能控性矩阵条件数: cond(Co) = %.2f\n', cond(Co));
fprintf('行列式: det(Co) = %.4f\n', det(Co));

% 物理解释
fprintf('\n物理诠释：\n');
fprintf('  小车加速度ü可直接影响ẍ（小车加速度→小车速度→小车位移），\n');
fprintf('  同时通过耦合项影响摆杆角加速度φ̈，进而影响φ和φ̇。\n');
fprintf('  因此ü能"触及"全部4个状态变量，系统完全能控。\n\n');

%% 3.3 能观性定义与基本概念
fprintf('=== 3.3 能观性定义与基本概念 ===\n');
fprintf('能观性定义：对于线性定常系统，若在有限时间[t₀, t_f]内，根据\n');
fprintf('系统的输出y(t)和控制输入u(t)的测量值，能唯一确定系统的初始\n');
fprintf('状态z(t₀)，则称系统状态完全能观。\n\n');
fprintf('物理意义：仅测量小车位移x和摆角φ，能否推算出ẋ和φ̇？\n\n');

%% 3.4 能观性的代数秩判据
fprintf('=== 3.4 能观性的代数秩判据 ===\n');

% 构造能观性判别矩阵 Ob = [C; CA; CA²; CA³]
fprintf('--- 3.4.1 能观性判别矩阵构造 ---\n');
Ob = obsv(A_full, C);
fprintf('Ob = [C; CA; CA²; CA³] (8×4):\n');
disp(Ob);

% 计算秩
fprintf('--- 3.4.2 矩阵秩计算 ---\n');
rank_Ob = rank(Ob);
fprintf('rank(Ob) = %d\n', rank_Ob);

if rank_Ob == 4
    fprintf('rank(Ob) = 4 = n（系统阶数）→ 系统完全能观 ✓\n');
else
    fprintf('rank(Ob) = %d < 4 → 系统不完全能观 ✗\n', rank_Ob);
end

% 物理解释
fprintf('\n物理诠释：\n');
fprintf('  通过位移传感器测x，通过编码器测φ。\n');
fprintf('  从x的测量值可重构ẋ（微分或观测器），\n');
fprintf('  从φ的测量值可重构φ̇。\n');
fprintf('  因此两个传感器足以观测全部4个状态，系统完全能观。\n\n');

%% 3.5 能控能观性的工程意义
fprintf('=== 3.5 能控能观性的工程意义 ===\n');
fprintf('(1) 能控性 → 状态反馈设计的必要条件：\n');
fprintf('    只有系统完全能控，才能通过状态反馈u = -Kz任意配置\n');
fprintf('    闭环极点位置。本系统满足此条件，可进行极点配置设计。\n\n');
fprintf('(2) 能观性 → 状态观测器设计的必要条件：\n');
fprintf('    只有系统完全能观，才能设计状态观测器从输出y重构\n');
fprintf('    全维状态ẑ。本系统满足此条件，可设计Luenberger观测器。\n\n');
fprintf('(3) 分离原理的前提：\n');
fprintf('    能控性+能观性同时成立，状态反馈与状态观测器可独立设计，\n');
fprintf('    闭环系统的极点 = 控制器极点 ∪ 观测器极点。\n\n');

%% 保存结果
save('co_params.mat', 'Co', 'rank_Co', 'Ob', 'rank_Ob');
fprintf('能控能观性分析完成。参数已保存至 co_params.mat\n');
