# -*- coding: utf-8 -*-
import numpy as np
from neural_decoding.PSID_2D.lib.PSID.PSID import filter_update
from collections import deque
from multiprocessing import shared_memory
from neural_decoding.PSID_2D.decoder import PSIDDecoder2D

class PSIDDecoder3D(PSIDDecoder2D):
    """
    2D速度 + 点击概率/二值。
    约定 Z = [pos_x, pos_y, vel_x, vel_y, click, 1]
    预测返回 [vel_x, vel_y, click]，并将 click 按 0.5 阈值二值化。
    """
    def predict(self, X_test):
        # 初始化共享门控
        if getattr(self, '_shm_flag', 1) == 1:
            self._PosFlag = shared_memory.ShareableList(name='PosFlag')
            self._shm_flag = 0

        # 未就绪：初始化并返回 None（与 2D 保持一致）
        if self._PosFlag[0] == 0 or getattr(self, '_LatentState', None) is None or getattr(self, '_Pp', None) is None:
            if getattr(self, '_A', None) is None:
                return None
            self._LatentState = np.zeros((self._A.shape[0], 1))
            self._Pp = np.eye(self._A.shape[0])
            return None

        # 调用同一 filter_update
        self._LatentState, self._Pp, BehaviorRelative = filter_update(
            self._LatentState, self._Pp,
            X_test.squeeze()[self._select_neural],
            self._Cz,
            self._A,
            self._C,
            self._R,
            self._S,
            self._Q
        )
        out = BehaviorRelative.squeeze()[2:-1]
        # out 现在应为 [vel_x, vel_y, click_prob]
        if out.shape[-1] >= 3:
            click_prob = float(out[-1])
            click_bin = 1.0 if click_prob >= 0.5 else 0.0
            out[-1] = click_bin
        return out
