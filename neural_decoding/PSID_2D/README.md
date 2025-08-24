# Wiener Filter算法
## WienerDecoder

WienerDecoder有2个方法

### 1.fit
输入：
    X_train : T x N.firing rates.
    y_train : T x D. vel.

示例代码：
```python
import numpy as np
from decoder import WienerDecoder

X_train = np.random.rand(100,3)
y_train = np.random.rand(100,2)
M = 2
W = WienerDecoder(M)
W.fit(X_train,y_train)
print(W.filters)
```
输出：
```bibtex
[[ 0.17288236 -0.01386758]
 [-0.04334128  0.08281381]
 [-0.0948528   0.04972172]
 [-0.04285885 -0.01082495]
 [-0.13332545 -0.14145781]
 [ 0.00349358  0.11553666]
 [ 0.07524571 -0.15928858]
 [ 0.01060119 -0.08701442]
 [-0.04809004  0.08680672]
 [ 0.59226393  0.51974769]]

```

### 2.predict
输入:
    X_test : 1 x (M+1)*N+1 .firing rates from (M+1) points.
输出：
    pred_vel : 1 x D.Predicted vel.
```python
X_test = np.random.rand(1,10)
pred_vel = W.predict(X_test)
print(pred_vel)
```
输出结果：
```bibtex
[[0.45675132 0.3118072 ]]
```

