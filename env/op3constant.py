import numpy as np

knee = 30
hip_roll = 15 # side (left, right)
hip_yaw = 30
hip_pitch = 30 # front, back
ank_pitch = 30
ank_roll = 30

head_tilt = 0
sho_roll = 0 # shoulder down, up
sho_pitch = 0
el = 0

joint_ranges = [
    head_tilt, # head_tilt 0
    ank_pitch, # l_ank_pitch 1
    ank_roll, # l_ank_roll 2
    el, # l_el 3
    hip_pitch, # l_hip_pitch 4
    hip_roll, # l_hip_roll 5
    hip_yaw, # l_hip_yaw 6
    knee, # l_knee 7
    sho_pitch, # l_sho_pitch 8
    sho_roll, # l_sho_roll 9
    ank_pitch, # r_ank_pitch 3
    ank_roll, # r_ank_roll 11
    el, # r_el 12
    hip_pitch, # r_hip_pitch 13
    hip_roll, # r_hip_roll 14
    hip_yaw, # r_hip_yaw 15
    knee, # r_knee 16
    sho_pitch, # r_sho_pitch 17
    sho_roll, # r_sho_roll 18
]

hip_pitch_bias = 0 # front
knee_bias = 0 # human ordinary
sho_roll_bias = 60 # down
ank_pitch_bias = 0 # up
hip_roll_bias = 0 # wide

joint_bias = [
    0, # head_tilt 0
    ank_pitch_bias, # l_ank_pitch 1
    0, # l_ank_roll 2
    0, # l_el 3
    -hip_pitch_bias, # l_hip_pitch 4
    -hip_roll_bias, # l_hip_roll 5
    0, # l_hip_yaw 6
    knee_bias, # l_knee 7
    0, # l_sho_pitch 8
    sho_roll_bias, # l_sho_roll 9
    -ank_pitch_bias, # r_ank_pitch 10
    0, # r_ank_roll 11
    0, # r_el 12
    hip_pitch_bias, # r_hip_pitch 13
    hip_roll_bias, # r_hip_roll 14
    0, # r_hip_yaw 15
    -knee_bias, # r_knee 16
    0, # r_sho_pitch 17
    -sho_roll_bias, # r_sho_roll 18
]

op3_module_names = [
    'head_tilt',
    'l_ank_pitch',
    'l_ank_roll',
    'l_el',
    'l_hip_pitch',
    'l_hip_roll',
    'l_hip_yaw',
    'l_knee',
    'l_sho_pitch',
    'l_sho_roll',
    'r_ank_pitch',
    'r_ank_roll',
    'r_el',
    'r_hip_pitch',
    'r_hip_roll',
    'r_hip_yaw',
    'r_knee',
    'r_sho_pitch',
    'r_sho_roll',
]

op3_obs_module_names = [
    'l_ank_pitch',
    'l_ank_roll',
    'l_hip_pitch',
    'l_hip_roll',
    'l_hip_yaw',
    'l_knee',
    'r_ank_pitch',
    'r_ank_roll',
    'r_hip_pitch',
    'r_hip_roll',
    'r_hip_yaw',
    'r_knee',
]

controller_names = [
    "head_tilt_position",
    "l_ank_pitch_position",
    "l_ank_roll_position",
    "l_el_position",
    "l_hip_pitch_position",
    "l_hip_roll_position",
    "l_hip_yaw_position",
    "l_knee_position",
    "l_sho_pitch_position",
    "l_sho_roll_position",
    "r_ank_pitch_position",
    "r_ank_roll_position",
    "r_el_position",
    "r_hip_pitch_position",
    "r_hip_roll_position",
    "r_hip_yaw_position",
    "r_knee_position",
    "r_sho_pitch_position",
    "r_sho_roll_position"
]
