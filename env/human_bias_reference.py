import numpy as np

def symmetric(x):
    return [
        x[0],
        -x[10],
        -x[11],
        -x[12],
        -x[13],
        -x[14],
        -x[15],
        -x[16],
        -x[17],
        -x[18],
        -x[1],
        -x[2],
        -x[3],
        -x[4],
        -x[5],
        -x[6],
        -x[7],
        -x[8],
        -x[9],
    ]

keyframes = np.array([
    [
        0, # head_tilt 0
        0, # l_ank_pitch 1
        0, # l_ank_roll 2
        0, # l_el 3
        -0, # l_hip_pitch 4
        -0, # l_hip_roll 5
        0, # l_hip_yaw 6
        0, # l_knee 7
        0, # l_sho_pitch 8
        0, # l_sho_roll 9
        -0, # r_ank_pitch 10
        0, # r_ank_roll 11
        0, # r_el 12
        0, # r_hip_pitch 13
        0, # r_hip_roll 14
        0, # r_hip_yaw 15
        -0, # r_knee 16
        0, # r_sho_pitch 17
        -0, # r_sho_roll 18
    ],
    [
        0, # head_tilt 0
        30, # l_ank_pitch 1
        0, # l_ank_roll 2
        0, # l_el 3
        -90, # l_hip_pitch 4
        -0, # l_hip_roll 5
        0, # l_hip_yaw 6
        60, # l_knee 7
        0, # l_sho_pitch 8
        0, # l_sho_roll 9
        -30, # r_ank_pitch 3
        0, # r_ank_roll 11
        0, # r_el 12
        -10, # r_hip_pitch 13
        0, # r_hip_roll 14
        0, # r_hip_yaw 15
        -60, # r_knee 16
        0, # r_sho_pitch 17
        -0, # r_sho_roll 18
    ],
], dtype=np.float64)

keyframes = np.array([keyframes[0], keyframes[1], keyframes[0], symmetric(keyframes[1])])
keyframes = keyframes * np.pi / 180.0
