# MATLAB Control Systems Coursework

MATLAB 控制系统课程作业，覆盖经典控制与现代控制的常见分析、建模和设计流程。

## 模块

### 经典控制

- 系统建模
- 时域响应分析
- 频域分析
- 根轨迹设计

### 现代控制

- 状态空间建模
- 状态方程求解
- 能控性与能观性分析
- 状态反馈设计

## 使用方法

使用 MATLAB 打开对应模块的入口脚本：

```matlab
main_run
main_run_module2
```

建议使用与课程环境兼容的 MATLAB 版本运行。

## 参数文件

源码依赖本地 `.mat` 参数文件。由于当前上传通道仅支持文本文件，以下文件需要通过 GitHub 网页补传到相应目录：

```text
classic-control/freq_params.mat
classic-control/model_params.mat
classic-control/pd_params.mat
classic-control/rl_params.mat
modern-control/co_params.mat
modern-control/sf_params.mat
modern-control/ss_params.mat
modern-control/state_sol_params.mat
```

## 目录结构

```text
classic-control/   # 经典控制模块
modern-control/    # 现代控制模块
```
