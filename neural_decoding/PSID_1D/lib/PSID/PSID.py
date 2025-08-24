""" 
Copyright (c) 2020 University of Southern California
See full notice in LICENSE.md
Omid G. Sani and Maryam M. Shanechi
Shanechi Lab, University of Southern California
"""

import numpy as np
from scipy import linalg
from .LSSM import LSSM
import time

'''
def projOrth(A, B): #cost time
    """
    Projects A onto B. A and B must be wide matrices with dim x samples.
    Returns:
    1) AHat: projection of A onto B
    2) W: The matrix that gives AHat when it is right multiplied by B
    """
    if B is not None:
        BCov = B @ B.T / B.shape[0]  # Division by num samples eventually cancels out but it makes the computations (specially pinv) numerically more stable
        ABCrossCov = A @ B.T / B.shape[0]
        isOk, attempts = False, 0
        #BCov_d = cupy.array(BCov)
        while not isOk and attempts < 10:
            try:
                attempts += 1
                #W = ABCrossCov @ cupy.asnumpy(cupy.linalg.pinv(BCov_d))
                W = ABCrossCov @ np.linalg.pinv(BCov) # or: A / B = A * B.' * pinv(B * B.')
                isOk = True
            except Exception as e:
                print('Error: "{}". Will retry...'.format(e))
        if not isOk:
            raise(Exception(e))
        AHat = W @ B  # or: A * B.' * pinv(B * B.') * B
    else:
        W = np.zeros( (A.shape[0], B.shape[1]) )
        AHat = np.zeros( A.shape )
    #free_gpu_memory()
    return (AHat, W)'''

def projOrth(A, B): #cost time
    """
    Projects A onto B. A and B must be wide matrices with dim x samples.
    Returns:
    1) AHat: projection of A onto B
    2) W: The matrix that gives AHat when it is right multiplied by B
    """
    if B is not None:
        BCov = B @ B.T / B.shape[0]  # Division by num samples eventually cancels out but it makes the computations (specially pinv) numerically more stable
        ABCrossCov = A @ B.T / B.shape[0]
        isOk, attempts = False, 0
        #BCov_d = cupy.array(BCov)
        while not isOk and attempts < 10:
            try:
                attempts += 1
                #W = ABCrossCov @ cupy.asnumpy(cupy.linalg.pinv(BCov_d))
                W = ABCrossCov @ np.linalg.pinv(BCov) # or: A / B = A * B.' * pinv(B * B.')
                isOk = True
            except Exception as e:
                print('Error: "{}". Will retry...'.format(e))
        if not isOk:
            raise(Exception(e))
        AHat = W @ B  # or: A * B.' * pinv(B * B.') * B
    else:
        W = np.zeros( (A.shape[0], B.shape[1]) )
        AHat = np.zeros( A.shape )
    #free_gpu_memory()
    return (AHat, W)

def blkhankskip(Y, i, j=None, s=0, time_first=True):
    """
    Constructs block Hankel matrices from the provided data Y
    """  
    N,ny = Y.shape
    if j == None: 
        j = N - i + 1
    H = np.empty((ny * i, j))
    
    
    for r in range(i):
        H[slice(r*ny, r*ny + ny), :] = Y[slice(s+r, s+r+j), :].T
    
    return H

def getHSize(Y, i, time_first=True):
    """
    Extracts time and data dimension information and the expected size of 
    the block Hankel matrices that will be constructed using blkhankskip
    """
    ny = None
    y1 = None
    if not isinstance(Y, (list, tuple)):
        if time_first:
            ySamples, ny = Y.shape
        else:
            ny, ySamples = Y.shape
        N = ySamples - 2*i + 1
        if ySamples > 0:
            y1 = Y.flatten()[0]
    else:
        ySamples = []
        N = []
        for yi, thisY in enumerate(Y):
            nyThis, ySamplesThis, NThis, y1This = getHSize(thisY, i, time_first)
            if yi == 0:
                ny = nyThis
                y1 = y1This
            else:
                if nyThis != ny:
                    raise(Exception('Size of dimension 1 must be the same in all elements of the data list.'))
            ySamples.append(ySamplesThis)
            N.append(NThis)
    return ny, ySamples, N, y1

def fitCzViaKFRegression(s, Y, Z, time_first):
    """
    Fits the behavior projection parameter Cz by first estimating 
    the latent states with a Kalman filter and then using ordinary 
    least squares regression
    """
    if not isinstance(Y, (list, tuple)):
        if time_first:
            YTF = Y
            ZTF = Z
        else:
            YTF = Y.T
            ZTF = Z.T
        xHat = s.kalman(YTF)[0]
    else:
        for yInd in range(len(Y)):
            if time_first:
                YTFThis = Y[yInd]
                ZTFThis = Z[yInd]
            else:
                YTFThis = Y[yInd].T
                ZTFThis = Z[yInd].T
            xHatThis = s.kalman(YTFThis)[0]
            if yInd == 0:
                xHat = xHatThis
                ZTF = ZTFThis
            else:
                xHat = np.concatenate( (xHat, xHatThis), axis=0)
                ZTF = np.concatenate( (ZTF, ZTFThis), axis=0)
    Cz = projOrth(ZTF.T, xHat.T)[1]
    return Cz


def PSID(Y, Z=None, nx=6, n1=5, i=4, WS=dict(), return_WS=False, fit_Cz_via_KF=False, time_first=True):
    ny, ySamples, N, y1 = getHSize(Y, i, time_first=time_first)
    if Z is not None:
        nz, zSamples, _, z1 = getHSize(Z, i, time_first=time_first)
    else:
        nz, zSamples = 0, 0

    if 'N' in WS and WS['N'] == N and \
        'i' in WS and WS['i'] == i and \
        'ySamples' in WS and WS['ySamples'] == ySamples and \
        'zSamples' in WS and WS['zSamples'] == zSamples and \
        'Y1' in WS and WS['Y1'] == y1 and \
        (nz == 0 or ('Z1' in WS and WS['Z1'] == z1)):
        # Have WS from previous call with the same data
        pass
    else:
        WS = {
            'N': N,
            'i': i,
            'ySamples': ySamples,
            'Y1': y1
        }
        if nz > 0:
            WS['zSamples'] = zSamples
            WS['Z1'] = z1

    if 'Yp' not in WS or WS['Yp'] is None:
        WS['Yp'] = blkhankskip(Y, i, N, time_first=time_first)
        WS['Yii'] = blkhankskip(Y, 1, N, i, time_first=time_first)
        if nz > 0:
            WS['Zii'] = blkhankskip(Z, 1, N, i, time_first=time_first)
    
    if n1 > nx:
        n1 = nx  # n1 can at most be nx

    # Stage 1
    if n1 > 0 and nz > 0:
        if n1 > i*nz:
            raise(Exception('n1 (currently {}) must be at most i*nz={}*{}={}. Use a larger horizon i.'.format(n1,i,nz,i*nz)))
        if 'ZHat_U' not in WS or WS['ZHat_U'] is None:
            Zf = blkhankskip(Z, i, N, i, time_first=time_first)
            WS['ZHat'] = projOrth(Zf, WS['Yp'])[0] # Zf @ WS['Yp'].T @ np.linalg.pinv(WS['Yp'] @ WS['Yp'].T) @ WS['Yp']  # Eq. (10)
            Yp_Plus = np.concatenate((WS['Yp'], WS['Yii']))
            Zf_Minus = Zf[nz:, :]
            WS['ZHatMinus'] = projOrth(Zf_Minus, Yp_Plus)[0] # Zf_Minus @ Yp_Plus.T @ np.linalg.pinv(Yp_Plus @ Yp_Plus.T) @ Yp_Plus  # Eq. (11)            

            # Take SVD of ZHat
            #(U_d,S_d,Z_d) = cupy.linalg.svd(cupy.array(WS['ZHat']), full_matrices=False) 
            #(WS['ZHat_U'], WS['ZHat_S'], ZHat_V) = (cupy.asnumpy(U_d),cupy.asnumpy(S_d),cupy.asnumpy(Z_d))
            WS['ZHat_U'], WS['ZHat_S'], ZHat_V = linalg.svd(WS['ZHat'], full_matrices=False, lapack_driver='gesvd')     # Eq. (12)

        Sz = np.diag(WS['ZHat_S'][:n1])        # Eq. (12)
        Uz = WS['ZHat_U'][:  , :n1]            # Eq. (12)

        Oz = Uz @ Sz**(1/2)                    # Eq. (13)
        Oz_Minus = Oz[:-nz, :]                 # Eq. (15)

        Xk = np.linalg.pinv(Oz) @ WS['ZHat'];                    # Eq. (14)
        Xk_Plus1 = np.linalg.pinv(Oz_Minus) @ WS['ZHatMinus'];   # Eq. (16)
    else:
        n1 = 0
        Xk = np.empty([0, N])
        Xk_Plus1 = np.empty([0, N])
    
    # Stage 2
    n2 = nx - n1     
    if n2 > 0:
        if nx > i*ny:
            raise(Exception('nx (currently {}) must be at most i*ny={}*{}={}. Use a larger horizon i.'.format(nx,i,ny,i*ny)))
        if 'YHat_U' not in WS or WS['YHat_U'] is None or \
            'n1' not in WS or WS['n1'] != n1:
            WS['n1'] = n1
            
            Yf = blkhankskip(Y, i, N, i, time_first=time_first)
            Yf_Minus = Yf[ny:, :]

            if n1 > 0: # Have already extracted some states, so remove the already predicted part of Yf
                # Remove the already predicted part of future y
                Oy1 = projOrth(Yf, Xk)[1] # Yf @ Xk.T @ np.linalg.pinv(Xk @ Xk.T)  # Eq. (18) - Find the y observability matrix for Xk
                Yf = Yf - Oy1 @ Xk                           # Eq. (19)

                Oy1_Minus = Oy1[:-ny, :]                     # Eq. (20)
                Yf_Minus = Yf_Minus - Oy1_Minus @ Xk_Plus1   # Eq. (21)
            
            WS['YHat'] = projOrth(Yf, WS['Yp'])[0] # Yf @ WS['Yp'].T @ np.linalg.pinv(WS['Yp'] @ WS['Yp'].T) @ WS['Yp']
            Yp_Plus = np.concatenate((WS['Yp'], WS['Yii']))
            WS['YHatMinus'] = projOrth(Yf_Minus, Yp_Plus)[0] # Yf_Minus @ Yp_Plus.T @ np.linalg.pinv(Yp_Plus @ Yp_Plus.T) @ Yp_Plus  # Eq. (23)

            # Take SVD of YHat
            WS['YHat_U'], WS['YHat_S'], YHat_V = linalg.svd(WS['YHat'], full_matrices=False,lapack_driver = 'gesvd')     # Eq. (24) cost time
            #WS['YHat_U'], WS['YHat_S'], YHat_V = linalg.svd(WS['YHat'], full_matrices=False)
            # free_gpu_memory()
        S2 = np.diag(WS['YHat_S'][:n2])              # Eq. (24)
        U2 = WS['YHat_U'][:  , :n2]                  # Eq. (24)
    
        Oy = U2 @ S2**(1/2)                          # Eq. (25)
        Oy_Minus = Oy[:-ny, :]                       # Eq. (27)

        Xk2 = np.linalg.pinv(Oy) @ WS['YHat'];                    # Eq. (26)
        Xk2_Plus1 = np.linalg.pinv(Oy_Minus) @ WS['YHatMinus'];   # Eq. (28)

        Xk = np.concatenate((Xk, Xk2))                            # Eq. (29)
        Xk_Plus1 = np.concatenate((Xk_Plus1, Xk2_Plus1))          # Eq. (29)

    
    # Parameter identification
    if n1 > 0:
        # A associated with the z-related states
        A = projOrth( Xk_Plus1[:n1, :], Xk[:n1, :] )[1]              # Eq. (17)
    else:
        A = np.empty([0, 0])
    
    if n2 > 0:
        A23 = projOrth(Xk_Plus1[n1:, :], Xk)[1] # Xk_Plus1[n1:, :] @ Xk.T @ np.linalg.pinv(Xk @ Xk.T)   # Eq. (30)
        if n1 > 0:
            A10 = np.concatenate((A, np.zeros([n1,n2])), axis=1)
            A = np.concatenate((A10, A23))   # Eq. (31)
        else:
            A = A23

    w = Xk_Plus1 -  A @ Xk                         # Eq. (34)

    if nz > 0:
        Cz = projOrth(WS['Zii'], Xk)[1] # WS['Zii'] @ Xk.T @ np.linalg.pinv(Xk @ Xk.T)    # Eq. (33)
    else:
        Cz = np.empty([0, nx])

    Cy = projOrth(WS['Yii'], Xk)[1] # WS['Yii'] @ Xk.T @ np.linalg.pinv(Xk @ Xk.T)    # Eq. (32)
    v  = WS['Yii'] - Cy @ Xk                             # Eq. (34)

    # Compute noise covariances
    NA = w.shape[1]
    Q = (w @ w.T)/NA                                # Eq. (35)
    S = (w @ v.T)/NA                                # Eq. (35)
    R = (v @ v.T)/NA                                # Eq. (35)

    Q = (Q + Q.T)/2      # Make precisely symmetric
    R = (R + R.T)/2      # Make precisely symmetric

    s = LSSM(params = {
        'A': A,
        'C': Cy,
        'Q': Q,
        'R': R,
        'S': S
    })
    if fit_Cz_via_KF and nz > 0:
        Cz = fitCzViaKFRegression(s, Y, Z, time_first)
    s.Cz = Cz
    
    if not return_WS:
        return s
    else:
        return s, WS
    
def filter_update(Xp, Pp, Y,Cz,
                  A, C, R, S, Q, FeedForward=False):
    
    if FeedForward:
        newXp = A @ Xp
        Pp = A @ Pp @ A.T
        X = newXp
        time.sleep(0.02)
    else:
        zi = Y[:, np.newaxis] - C @ Xp # Innovation Z(i)
            # predict and correct
        ziCov = C @ Pp @ C.T + R
        Kf = np.linalg.lstsq(ziCov.T, (Pp @ C.T).T, rcond=None)[0].T  # Kf(i)
        Kw = np.linalg.lstsq(ziCov.T, S.T, rcond=None)[0].T   # Kw(i)
        
        #free_gpu_memory()
        K = A @ Kf + Kw                    # K(i)
    
        X = Xp + Kf @ zi # X(i|i)
    
        newXp = A @ Xp
        newXp += K@zi
    
        Pp = A @ Pp @ A.T + Q - K @ ziCov @ K.T
        
    return newXp, Pp, (Cz @ X).T
