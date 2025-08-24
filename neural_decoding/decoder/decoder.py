# Registry mapping algorithm keys to decoder classes
from neural_decoding.PSID_2D.decoder import PSIDDecoder2D
from neural_decoding.PSID_3D.decoder import PSIDDecoder3D
from neural_decoding.refit_kalman_lcy_2D.decoder import RefitKF2D

decoder = {
    'PSID_2D': PSIDDecoder2D,
    'PSID_3D': PSIDDecoder3D,
    'refit_kf_lcy_2d': RefitKF2D,
    # add more keys if needed
}
