PSID_3D — 2D velocity + click decoder

## 概览

本模块提供基于 PSID 的 3 维行为解码器：在 2D 速度基础上增加“点击”输出（click or not）。

- 核心思路：沿用现有 `PSID_2D` 的状态空间辨识流程（学习 A, C, Q, R, S, Cz），将行为向量 Z 扩为包含 click 的版本；预测时在 `[vel_x, vel_y, click_prob]` 的第三维上做阈值化输出二值点击。
- 兼容 ROS2 现有节点：通过算法键 `'PSID_3D'` 选择该解码器，无需改动节点代码。

## 行为向量约定（Z）

- 训练与预测时，行为向量 Z 的维度与顺序采用：
	- `[pos_x, pos_y, vel_x, vel_y, click, 1]`
	- 末尾 `1` 为常量项（bias），与 `PSID_2D` 约定一致
	- click 可为 0/1 或概率（连续值），训练按回归处理

## 输出定义

- 预测输出为 3 维：`[vel_x, vel_y, click_bin]`
- 阈值化规则：`click_bin = 1.0 if click_prob >= 0.5 else 0.0`

## 代码位置与结构

- 新增类：`neural_decoding/PSID_3D/decoder.py`
	- 类名：`PSIDDecoder3D`
	- 继承自 `neural_decoding/PSID_2D/decoder.py` 的 `PSIDDecoder2D`
	- 仅重写 `predict()`，对 `BehaviorRelative.squeeze()[2:-1]` 的第三维做 0.5 阈值化

- 算法注册表：`neural_decoding/decoder/decoder.py`
	- 暴露 `decoder` 字典映射：
		- `'PSID_2D' -> PSIDDecoder2D`
		- `'PSID_3D' -> PSIDDecoder3D`
		- `'refit_kf_lcy_2d' -> RefitKF2D`
	- ROS 节点通过 `from neural_decoding.decoder.decoder import decoder` 获取类

## 训练/预测数据形状约定

- 训练追加：`update(X_train, y_train)`
	- X_train: `(N,)` 或 `(T, N)` 神经观测（发放率等）
	- y_train: `(D,)` 或 `(T, D)` 行为（按上述 Z 顺序组织）
	- 满足累计样本数 `> 50` 后，`fit()` 才会产出可用参数
	- 通道选择掩码：`select_neural = (Y.sum(0) > 2)`（与 `PSID_2D` 一致）

- 预测：`predict(X_test)`
	- 输入：单帧 `(1, N)`（内部会 `squeeze()` 并按掩码筛选通道）
	- 输出：`np.ndarray`，形如 `[vel_x, vel_y, click_bin]`
	- 共享门控：使用共享内存 `PosFlag`；`PosFlag[0] == 0` 或未初始化时返回 `None`

### 输入消息（ROS 对应关系）

- 训练阶段（样本积累 → update/fit）：使用 `interfaces/Sample.msg`
	- `y_observation: float64[]` → 神经观测 Y（长度 N）
	- `x_state: float64[]` → 行为 Z（长度 6，顺序 `[px, py, vx, vy, click, 1]`）
	- 多帧后组成矩阵：Y ∈ R[T, N]，Z ∈ R[T, 6]（T>50 再触发 `fit()`）

- 预测阶段（单帧 → predict）：使用 `interfaces/PassiveObservation.msg`
	- `y_observation: float64[]` → 神经观测 Y（长度 N），内部转为 `(1, N)`
	- `y_shape: int64[]` → 可忽略（当前未使用）
	- 输出发布到 `interfaces/State.msg` 的 `x_state: float64[]`，内容为 `[vel_x, vel_y, click_bin]`

## 与 PSID_2D 的关系

- 训练：完全沿用 `PSID_2D` 的 `PSID(Y[:, mask], Z)` 流程，只要 Z 中包含 click 维度即可一并学习到投影 `Cz`
- 预测：同样调用 `filter_update(...)` 得到 `(Cz @ X).T`，再在 `[2:-1]` 切片上阈值化第三维
- 行为维扩展后，`[2:-1]` 从原本 2 维（速度）变为 3 维（速度 + 点击）

## 与 RefitKF（refit_kf_lcy_2d）的差异（简述）

- PSID：子空间辨识 + 卡尔曼滤波，训练复杂度较高，能分离行为相关潜状态；
- RefitKF：显式线性回归 + KF，训练/预测开销低，但模型结构更依赖先验设定；
- 本模块仅影响 PSID 路线，不改变 RefitKF 的使用方式。

## 使用方法（ROS2 节点）

1) 确保注册表可用：`neural_decoding/decoder/decoder.py` 存在。
2) 在训练器/预测器参数中设置：`algorithm = 'PSID_3D'`。
3) 训练阶段，向 `x_state` 写入 `[pos_x, pos_y, vel_x, vel_y, click, 1]`；
4) 预测阶段，从 `State.x_state` 读取 3 维输出 `[vx, vy, click_bin]`；
5) 若下游仅需 2D 速度，取前两维；需要点击则读取第三维。

## 共享内存与门控

- `PosFlag`（name='PosFlag'）：0 表示未就绪，此时 `predict()` 返回 `None`；
- 与 `PSID_2D` 一致，先启动数据/训练再启动预测，或在启动前设置好门控；
- 该模块不使用 `SmoothCoefficient`（那是 RefitKF 的在线平滑参数）。

## 代码改动记录（此次变更）

- 新增文件：
	- `neural_decoding/PSID_3D/__init__.py`（导出 `PSIDDecoder3D`）
	- `neural_decoding/PSID_3D/decoder.py`（实现 3D 输出与点击阈值化）
	- `neural_decoding/decoder/decoder.py`（算法键到类的注册表）

- 现有节点无需改动：
	- `src/decoding_element/decoding_element/decoding_element_model_trainer.py`
	- `src/decoding_element/decoding_element/decoding_element_model_predictor.py`
	- 因它们只依赖注册表与算法键字符串。

## 最小示例（纯 Python，伪代码）

```python
from neural_decoding.PSID_3D.decoder import PSIDDecoder3D
import numpy as np

dec = PSIDDecoder3D()

# 伪造训练数据（T, N）与（T, D），D=6: [px, py, vx, vy, click, 1]
T, N = 200, 64
Y = np.random.poisson(0.2, size=(T, N)).astype(float)
Z = np.zeros((T, 6))
Z[:, 2] = np.random.randn(T)   # vx
Z[:, 3] = np.random.randn(T)   # vy
Z[:, 4] = (np.random.rand(T) > 0.7).astype(float)  # click 0/1
Z[:, 5] = 1.0                   # 常量项

for t in range(T):
		dec.update(Y[t], Z[t])

dec.fit()

# 单帧预测
X1 = Y[-1][np.newaxis, :]
pred = dec.predict(X1)  # [vx, vy, click_bin]
print(pred)
```

> 注：实际运行时需确保共享内存 `PosFlag`=1，否则 `predict()` 将返回 `None`（与 ROS2 运行时一致）。

## 故障排查

- 预测输出为 None：检查 `PosFlag` 是否为 1，或是否已收到训练发布的参数；
- 维度错误：确认 Z 顺序是否为 `[px, py, vx, vy, click, 1]`，以及通道掩码（训练/预测的神经通道顺序一致）；
- 导入失败：确认 `neural_decoding/decoder/decoder.py` 存在，且键 `'PSID_3D'` 可用。

