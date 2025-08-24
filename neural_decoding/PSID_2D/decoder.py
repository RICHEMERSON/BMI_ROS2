# -*- coding: utf-8 -*-            
# @Time : 2022/7/2 3:00 PM
#  :ljf
# @FileName: decoder.py
# @Software: PyCharm
import numpy as np
from .lib.PSID import PSID
from .lib.PSID.PSID import filter_update
from collections import deque
from multiprocessing import shared_memory

class PSIDDecoder2D(object):
    
    def __init__(self):
        self._training_neural_data = deque(maxlen=10000)
        self._training_state = deque(maxlen=10000)
        self._LSSMDecoder = None    
        self._LatentState = None
        self._Pp = None
        self._select_neural = None
        self._shm_flag = 1
        self._train_flag=1
    
    def update(self,X_train,y_train):
        self._training_neural_data.append(X_train)
        self._training_state.append(y_train)
    
    def fit(self):
        """
        Fit training data.

        Parameters
        ----------
        X_train :array-like,TxN.
            Firing rates.
        y_train :array-like,TxD.
            vel
        M : int.
            M+1 is the number of filters.
        """
        
        Y = np.array(self._training_neural_data)
        Z = np.array(self._training_state)
        
        if Y.shape[0]>50:
            
            if self._select_neural is None:
                self._select_neural = Y.sum(0)>2
            
            self._LSSMDecoder = PSID(Y = Y[:,self._select_neural], Z = Z)
            self._A = self._LSSMDecoder.A
            self._C = self._LSSMDecoder.C
            self._Q = self._LSSMDecoder.Q
            self._R = self._LSSMDecoder.R
            self._S = self._LSSMDecoder.S
            self._Cz = self._LSSMDecoder.Cz
            
            self._train_flag=0


    def predict (self,X_test):
        """
        Predict

        Parameters
        ----------
        X_test : array-like,(M+1)*N x 1
            Firing rates of one time point

        Returns
        --------
        pred_vel : array-like,1x2.
            predicted vel.

        """
        if self._shm_flag==1:
            self._PosFlag = shared_memory.ShareableList(name='PosFlag')
            self._shm_flag = 0
            
        if self._PosFlag[0]==0 or self._LatentState is None or self._Pp is None:
            self._LatentState = np.zeros((self._A.shape[0],1))
            self._Pp = np.eye(self._A.shape[0])
            return None
        
        self._LatentState, self._Pp, BehaviorRelative = filter_update(self._LatentState, self._Pp, 
                                                              X_test.squeeze()[self._select_neural],
                                                              self._Cz, 
                                                              self._A, 
                                                              self._C, 
                                                              self._R, 
                                                              self._S, 
                                                              self._Q)
         
            
        return BehaviorRelative.squeeze()[2:-1]
    
    def model_update(self, par):
        
        self._A = par['A']
        self._C = par['C']
        self._Q = par['Q']
        self._R = par['R']
        self._S = par['S']
        self._Cz = par['Cz']
        self._select_neural = par['select_neural']
    
    def model_par(self):
        par = {}
        par['A'] = self._A
        par['C'] = self._C
        par['Q'] = self._Q
        par['R'] = self._R
        par['S'] = self._S
        par['Cz'] = self._Cz
        par['select_neural'] = self._select_neural
         
        return par if self._train_flag==0 else None


