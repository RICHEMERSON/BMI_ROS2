# ReFit Kalman Filter
Based on kalman filter, refit kalman filter constrain transient matrix A,state covariance W. As a result,the prior error covariance is constrained.

So, the position don't influence velocity.



**State transient matrix A**
$$
A = \left[\begin{matrix}
1 &0 &dt &0 &0\\
    0 &1 &0 &dt & 0\\
    0 &0 &a_{vel_{horiz},vel_{horiz}} &a_{vel_{horiz},vel_{vert}} &0\\
    0 &0 &a_{vel_{vert},vel_{horiz}} &a_{vel_{vert},vel_{vert}} &0\\
    0 &0 &0 &0 &1
    
\end{matrix}\right]
$$
**State transient covariance W**
$$
W = 
\left[\begin{matrix}
0 &0 &0 &0 &0\\
0 &0 &0 &0 &0\\
0 &0 &W_{vel_{horiz},vel_{horiz}} &W_{vel_{horiz},vel_{vert}} &0\\
0 &0 &W_{vel_{vert},vel_{horiz}} &W_{vel_{vert},vel_{vert}} &0\\
0 &0 &0 &0 &0\\
\end{matrix}\right]
$$
**Prior error covariance**  
$$
\hat{p}^-_k = 
\left[\begin{matrix}
\pmb{0} &\pmb{0} &0\\
\pmb{0} &\Sigma^{v,v}_{t|t-1} &0\\
0 &0 &0
\end{matrix}\right]
$$
However,I have encountered some problem,so the results is poor. In the *Refit kalman filter* paper,the position is true,posterior position.

Refit kalman filter has 3 interfaces.

## 1. fit

参数: 

   last_x : array_like, Dx1. State at last step.

   last_covar : array_like, DxD. State error covariance at last step.

输入：

​			dt : float, time bin width.

​			X_train: T x N,firing rates.    

​			y_train: T x D,vel.

```
from decoder import REFITKALMANDECODERUSER
from lib.ReFit_kalman_online import REFITKALMAN
import numpy as np
dt = 0.05
last_x = y_test[0,:]
last_covar = np.zeros((y_train.shape[1],y_train.shape[1]))
model = REFITKALMANDECODERUSER(dt,last_x,last_covar)
model.fit(y_train,X_train)
```

## 2.predict

输入：

​	X_test: array_like,Nx1,firing rates.

输出：

   predict_states: array_like,Dx1.

```
predict_states = model.predict(X_test).reshape(1,-1)
```

