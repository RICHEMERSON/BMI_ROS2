# -*- coding: utf-8 -*-            
# @Time : 2022/7/15 10:41 PM
#  :ljf
# @FileName: pipe_test.py.py
# @Software: PyCharm
# -*- coding: utf-8 -*-
# @Time : 2022/7/11 9:19 PM
#  :ljf
# @FileName: pipline.py
# @Software: PyCharm

import matplotlib.pyplot as plt
import numpy as np
from decoder import WienerDecoder
import pandas as pd
import random
from sklearn.model_selection import train_test_split
from nlb_tools.nwb_interface import NWBDataset

# 1. Load MZ data
# dataset = NWBDataset("./000128/sub-Jenkins", "*train", split_heldout=False)
# dataset.resample(50)
# dataset.smooth_spk(50, name='smth_50')
# trial_data = dataset.make_trial_data(align_field='move_onset_time', align_range=(-130, 370))
# lagged_trial_data = dataset.make_trial_data(align_field='move_onset_time', align_range=(-50, 450))

#训练用的数据。存储在extracted_data文件下。
# firing_rates = trial_data.spikes_smth_50.to_numpy()##提取rates
# vel = lagged_trial_data.hand_vel.to_numpy()#
# pos = lagged_trial_data.hand_pos.to_numpy()
firing_rates = np.load('./extracted_data/rates.npy')
vel = np.load('./extracted_data/vel.npy')
pos = np.load('./extracted_data/pos.npy')

# ## set some constants
one_trial_time = 10
trials = int(firing_rates.shape[0]/one_trial_time)
percentage = 0.9

# get train and test data
train_lable = random.sample( sorted(np.arange(trials)),int(trials*percentage))
test_lable = set(np.arange(trials)) - set(train_lable)
test_lable = list(test_lable)
reshaped_rates = firing_rates.reshape(trials,one_trial_time,-1)
reshaped_vel = vel.reshape(trials,one_trial_time,-1)
reshaped_pos = pos.reshape(trials,one_trial_time,-1)
X_train = reshaped_rates[train_lable,:,:].reshape(len(train_lable)*one_trial_time,-1)
y_train = reshaped_vel[train_lable,:,:].reshape(len(train_lable)*one_trial_time,-1)
X_test = reshaped_rates[test_lable,:,:].reshape(len(test_lable)*one_trial_time,-1)
y_test = reshaped_vel[test_lable,:,:].reshape(len(test_lable)*one_trial_time,-1)
true_test_pos = reshaped_pos[test_lable,:,:].reshape(len(test_lable)*one_trial_time,-1)

# 2.train  and predict wiener filter
M = 9 ## (M+1) filter numbers
#
# # train
model = WienerDecoder(M)
model.fit(X_train,y_train)
#
# get test data
(T_test,N) = X_test.shape
T_wiener_rates = np.zeros((T_test,(M+1)*N))
predicted_vel = np.zeros(( T_test,y_test.shape[1]))
for i in range(T_test):
    if i <M :
        t_wiener_rates = np.zeros((M + 1, N))
        t_wiener_rates[M - i:, :] = X_test[0:i+1, :]#替换
        T_wiener_rates[i, :] = t_wiener_rates.ravel()  # 替换
    else :
        T_wiener_rates[i , :] = X_test[i - M:i + 1, :].ravel()
T_wiener_rates = np.c_[T_wiener_rates,np.ones(T_test).T]# !!! add one column '1'
#
# #
for i in range(T_test):
    predicted_vel[i,:] = model.predict(T_wiener_rates[i,:])
#
cc =[np.corrcoef(y_test[:,0].reshape(1,-1),predicted_vel[:,0].reshape(1,-1))[0,1],\
     np.corrcoef(y_test[:,1].reshape(1,-1),predicted_vel[:,1].reshape(1,-1))[0,1]] # 0。87 ，0.84
print(cc)
# pd.DataFrame(predicted_vel).to_csv('./test/test_pred_vel.csv')

#  3. plot vel and predicted pos
one_trial_time = 10
test_trials = len(test_lable)
reshaped_predicted_vel = predicted_vel.reshape(test_trials,one_trial_time,-1)
# vel to pos
reshaped_pred_pos=np.zeros(((test_trials,one_trial_time,2)))
reshaped_pred_pos[:,0,:] = true_test_pos.reshape(test_trials,one_trial_time,-1)[:,0,:]#替换第一个原点位置
for i in range(test_trials):
    reshaped_pred_pos[i,:,:] = np.cumsum(reshaped_predicted_vel[i,:,:],axis=0)*50/1000

fig,axs = plt.subplots(2,3)
X = np.arange(0,10)*50/1000
for i in range(40):# 30 trials
    axs[0][0].plot( X , y_test[i*10:(i+1)*10,0])
    axs[1][0].plot( X , reshaped_predicted_vel[i, :, 0])
    axs[0][1].plot(X, y_test[i*10:(i+1)*10 ,1])
    axs[1][1].plot(X, reshaped_predicted_vel[i, :, 1])
    axs[0][2].plot(true_test_pos[i*10:(i+1)*10 ,0],true_test_pos[i*10:(i+1)*10,1])
    axs[1][2].plot(reshaped_pred_pos[i,:,0],reshaped_pred_pos[i,:,1])
plt.savefig('./test/40个trials.png')
plt.show()


