%% sys_modeling.m — 第1章：直线一级倒立摆系统建模
% GIP-100-L 型直线一级倒立摆
% 方法：Lagrange法 → 非线性方程 → 泰勒线性化 → 传递函数
% 控制输入：小车加速度 ü（对应实际系统的伺服电机电流环输出）

clear; clc;
disp('========================================');
disp('  第1章：控制系统建模');
disp('========================================');
fprintf('\n');

%% 1.2 系统物理参数定义
M = 0.5;      % 小车质量 [kg]
m = 0.2;      % 摆杆质量 [kg]
L = 0.6;      % 摆杆全长 [m]
l = L/2;      % 摆杆半长（质心到转轴距离）[m]
g = 9.8;      % 重力加速度 [m/s^2]

% 均质细杆绕质心的转动惯量
I_cm = (1/12) * m * L^2;   % I_cm = (1/12)*m*L^2
% 绕转轴(摆杆端点)的转动惯量（平行轴定理）
J = I_cm + m * l^2;         % J = I_cm + m*l^2

fprintf('=== 1.2 系统物理参数 ===\n');
fprintf('小车质量 M       = %.2f kg\n', M);
fprintf('摆杆质量 m       = %.2f kg\n', m);
fprintf('摆杆全长 L       = %.2f m\n', L);
fprintf('摆杆半长 l       = %.2f m\n', l);
fprintf('重力加速度 g     = %.2f m/s^2\n', g);
fprintf('绕质心转动惯量 I_cm = %.6f kg·m^2\n', I_cm);
fprintf('绕转轴转动惯量 J    = %.6f kg·m^2\n', J);
fprintf('\n');

%% 1.3 牛顿-欧拉/Lagrange动力学建模
% 非线性方程（Lagrange法推导）：
%   (M+m)·x'' + m·l·φ''·cos(φ) - m·l·(φ')^2·sin(φ) = F
%   J·φ'' + m·l·x''·cos(φ) - m·g·l·sin(φ) = 0
%
% 在GIP-100-L系统中，伺服电机具有高带宽电流环，
% 有效控制输入为小车加速度 ü（即 ẍ），而非力F。
% 这种简化的物理依据是：
%   1. 电机电流环的响应速度远快于机械系统
%   2. 将加速度作为虚拟控制量，实际力由电机电流环跟踪实现
%   3. 该简化在倒立摆控制中被广泛采用

fprintf('=== 1.3 非线性动力学方程 ===\n');
fprintf('(M+m)·ẍ + m·l·φ̈·cos(φ) - m·l·(φ̇)²·sin(φ) = F\n');
fprintf('J·φ̈ + m·l·ẍ·cos(φ) - m·g·l·sin(φ) = 0\n\n');

%% 1.4 非线性方程的线性化处理
% 在平衡点 φ=0 处线性化（cos(φ)≈1, sin(φ)≈φ, (φ')^2≈0）：
%   (M+m)·ẍ + m·l·φ̈ = F
%   J·φ̈ + m·l·ẍ - m·g·l·φ = 0
%
% 令控制输入 u = ẍ（小车加速度），从方程(2)得：
%   J·φ̈ + m·l·u - m·g·l·φ = 0
%   => φ̈ = (m·g·l/J)·φ - (m·l/J)·u
%
% 传递函数（加速度→摆角）：
%   Φ(s)/U(s) = -(m·l) / (J·s² - m·g·l)
%              = -(m·l/J) / (s² - m·g·l/J)

fprintf('=== 1.4 线性化（以加速度ü为输入）===\n');
fprintf('J·φ̈ + m·l·ü - m·g·l·φ = 0\n');
fprintf('=>  Φ(s)/U(s) = -(m·l/J) / (s² - m·g·l/J)\n\n');

%% 1.5 传递函数推导
fprintf('=== 1.5 传递函数 Phi(s)/U(s) ===\n');

% 摆角传递函数（加速度输入）
num_acc = -m*l;           % 分子: -ml
den_acc = [J, 0, -m*g*l]; % 分母: J·s² - m·g·l

% 或写成标准形式
omega0 = sqrt(m*g*l/J);   % 固有"频率"参数
K_acc = -m*l/J;           % 增益

fprintf('传递函数: Phi(s)/U(s) = %.4f / (s^2 - %.4f)\n', K_acc, omega0^2);
fprintf('           U(s) = 小车加速度 ẍ(s)\n\n');

% 构造传递函数对象
G_phi = tf(num_acc, den_acc);

fprintf('传递函数表达式:\n');
G_phi

%% 1.5.3 开环极点分析
poles_phi = pole(G_phi);
fprintf('\n=== 1.5.3 开环极点分析 ===\n');
fprintf('开环极点:\n');
for i = 1:length(poles_phi)
    fprintf('  p%d = %+.4f\n', i, real(poles_phi(i)));
end

rhp_count = sum(real(poles_phi) > 0);
fprintf('\n右半平面极点数 = %d\n', rhp_count);
fprintf('系统开环不稳定！极点位于 s = ±%.4f，其中一个在右半平面。\n', omega0);
fprintf('必须依赖闭环反馈实现镇定。\n');

% 构建完整状态空间模型（4阶），供现代控制部分参考
% 状态变量: z = [x, ẋ, φ, φ̇]ᵀ
% 控制输入: u = ẍ
A_full = [0, 1, 0,           0;
          0, 0, 0,           0;
          0, 0, 0,           1;
          0, 0, m*g*l/J,     0];

B_full = [0; 1; 0; -m*l/J];

fprintf('\n完整状态空间模型（4阶，含小车位移）:\n');
fprintf('状态: z = [x, ẋ, φ, φ̇]^T,  输入: u = ẍ\n');
fprintf('ż = A·z + B·u\n');

% 保存关键变量
save('model_params.mat', 'M', 'm', 'l', 'L', 'g', 'I_cm', 'J', ...
     'omega0', 'K_acc', 'G_phi', 'poles_phi', 'rhp_count', ...
     'A_full', 'B_full', 'num_acc', 'den_acc');

fprintf('\n系统建模完成。参数已保存至 model_params.mat\n');
