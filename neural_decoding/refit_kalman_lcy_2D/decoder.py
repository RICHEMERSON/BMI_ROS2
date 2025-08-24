#!/usr/bin/env python
# coding=utf-8

"""
@author: ljf
@license: cuilab
@contact: liaojf2021@ion.ac.cn
@file: decoder.py
@date: 2022/7/26 下午10:06
@desc: 
"""

from pykalman import KalmanFilter
import numpy as np
from collections import deque
from sklearn.linear_model import LinearRegression
from multiprocessing import shared_memory
import time

class RefitKF2D(object):
    
    """
    ReFIT kalman filter decoder.
    """
    
    def __init__(self):
        
        """
        Constructor.

        Parameters
        ----------
        dt : float ,time bin width.
        last_x : array_like, D x 1. States at k-1 time step.
        last_covar : array_like, D x D. Error covariance at k-1 step.
        """
        
        self._training_neural_data = deque(maxlen=100000)
        self._training_state = deque(maxlen=100000)
        
        self._state_matrix = None
        self._state_offset = None
        self._observation_matrix = None
        self._observation_offset = None
        self._DecoderKF = None
        self._state_mean = None
        self._state_covariance = None
        self._select_neural = None
        self._transition_covariance = None
        self._observation_covariance = None
        
        self._shm_flag = 1
        self._train_flag = 1
        self._pre_par = None
        
        self.__sc =  None
    def update(self,X_train,y_train):
        self._training_neural_data.append(X_train)
        self._training_state.append(y_train)
        
    def fit(self):
        """
        Get some constant parameters.

        Parameters
        ----------
        X_train : array_like,T x D.states
        y_train : array_like,T x N.Observations.
        """
        y = np.array(self._training_neural_data)
        X = np.array(self._training_state)
        self._state_matrix = np.zeros((X.shape[1],X.shape[1]))
        self._state_matrix[[0,1,-1],[0,1,-1]] = 1
            
        
        if len(X)>50:
            if self._select_neural is None:
                self._select_neural = y.sum(0)>1
                
            StateModel = LinearRegression().fit(X[0:-1], X[1::])
            self._state_matrix[[0,1],[2,3]] = StateModel.coef_[[0,1],[2,3]]
            self._state_matrix[2:4,2:4] = StateModel.coef_[2:4,2:4]

            self._state_offset = StateModel.intercept_
            
            ObservationModel = LinearRegression().fit(X,y[:,self._select_neural])
            self._observation_matrix = ObservationModel.coef_.squeeze()
            self._observation_offset = ObservationModel.intercept_
            
            self._transition_covariance = (X[1::]-StateModel.predict(X[0:-1])).T@(X[1::]-StateModel.predict(X[0:-1]))/len(X[1::])
            self._observation_covariance = (y[:,self._select_neural]-ObservationModel.predict(X)).T@\
                (y[:,self._select_neural]-ObservationModel.predict(X))/len(X)
            
            self._DecoderKF = KalmanFilter(transition_matrices = self._state_matrix,
                         observation_matrices = self._observation_matrix,
                         transition_covariance = self._transition_covariance,
                         observation_covariance = self._observation_covariance,
                         transition_offsets = self._state_offset,
                         observation_offsets = self._observation_offset,
                         initial_state_mean = np.zeros(self._state_matrix.shape),
                         initial_state_covariance = np.zeros(self._transition_covariance.shape))
            self._train_flag = 0

    def predict(self,X_test):
        """
        get posterior estimation at k time step.

        Parameters
        ---------
        X_test : array_like, N x 1.Notice X_test is observations.

        Returns
        --------
        posterior_x : array_like , D x 1.
        """
        if self._shm_flag==1:
            self._PosFlag = shared_memory.ShareableList(name='PosFlag')
            self._shm_flag = 0
            
        if self._PosFlag[0]==0 or self._state_mean is None or self._state_covariance is None:
            self._state_mean = np.zeros(self._state_offset.shape)
            self._state_covariance = np.zeros(self._transition_covariance.shape)
            return np.array([float(self._PosFlag[0]),float(self._PosFlag[0])])
        
        self._state_mean, next_state_covariance = self._DecoderKF.filter_update(transition_matrix = self._state_matrix,
                                                               observation = X_test.squeeze()[self._select_neural],
                                                               observation_matrix = self._observation_matrix,
                                                               transition_covariance = self._transition_covariance,
                                                               observation_covariance = self._observation_covariance,
                                                               transition_offset = self._state_offset,
                                                               observation_offset = self._observation_offset,
                                                               filtered_state_mean = self._state_mean,
                                                               filtered_state_covariance = self._state_covariance)
        
        self._state_covariance[2:4,2:4] = next_state_covariance[2:4,2:4]
        return self._state_mean.data[2:-1]
    
    def model_update(self, par):
        
        if self.__sc is None:
            self.__sc = shared_memory.ShareableList(name='SmoothCoefficient')  
        
        if not self._pre_par is None:
            for i in par:
                if i=='select_neural':
                    continue
                par[i] = par[i]* self.__sc[0]+(1- self.__sc[0])*self._pre_par[i]
                pass
        
        self._observation_matrix = par['observation_matrix']
        self._state_matrix = par['state_matrix']
        self._transition_covariance = par['transition_covariance']
        self._observation_covariance = par['observation_covariance']
        self._state_offset = par['state_offset']
        self._observation_offset = par['observation_offset']
        self._select_neural = par['select_neural']
        
        if self._DecoderKF is None:
            self._DecoderKF = KalmanFilter(transition_matrices = self._state_matrix,
                         observation_matrices = self._observation_matrix,
                         transition_covariance = self._transition_covariance,
                         observation_covariance = self._observation_covariance,
                         transition_offsets = self._state_offset,
                         observation_offsets = self._observation_offset,
                         initial_state_mean = np.zeros(self._state_matrix.shape),
                         initial_state_covariance = np.zeros(self._transition_covariance.shape))
            self._pre_par = par.copy()
         
    def model_par(self):
        par = {}
        par['observation_matrix'] = self._observation_matrix
        par['state_matrix'] = self._state_matrix
        par['transition_covariance'] = self._transition_covariance
        par['observation_covariance'] = self._observation_covariance
        par['state_offset'] = self._state_offset
        par['observation_offset'] = self._observation_offset
        par['select_neural'] = self._select_neural
         
        return par if self._train_flag==0 else None

        
        

