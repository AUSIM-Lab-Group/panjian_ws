# Planner Package Layout

`planner` 目录按论文和 baseline 边界划分。现有方法不混写在同一个包里，便于复现实验和对比消融。

```text
planner/
├── dwa_planner/          # 传统局部规划 baseline
├── mpc_dcbf/             # 原 MPC-DCBF/ACBF baseline
├── mpc_secbf/            # 本文 MPC-SECBF 控制器
├── semantic_guard/       # 本文语义 β、Guard 和日志
└── vomp_planner/         # 全局路径搜索
```

本文方法的核心代码只依赖两个新包：

- `mpc_secbf`：订阅 `/safety_margin/beta`，构造 `h_SEE = h_EE - beta_i`，并在 MPC 中加入 SECBF 约束。
- `semantic_guard`：根据类别和上下文计算 `beta_i = beta_bar(c_i) * mu(phi_i)`，执行 `beta_i <= h_EE - eta` Guard，并输出 `guard_log.csv`。

`swarm_test` 负责实验编排，不属于具体控制器实现；其中的 `secbf_planner.launch`、`exp_secbf_planner.launch` 和 `run_secbf_sim_experiments.py` 用来把本文两个包接入数值仿真和实车链路。
