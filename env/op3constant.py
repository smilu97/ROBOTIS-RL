def s(lst, indices):
    res = []
    for i in indices: res.append(lst[i])
    return res

indices = [1,4,5,6,7,8,9,10,13,14,15,16,17,18]

knee = 50
hip_roll = 10
hip_yaw = 10
hip_pitch = 50
ank_pitch = 30

head_tilt = 0
sho_roll = 50
sho_pitch = 50
ank_roll = 5
el = 10

joint_ranges = s([
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
], indices)

op3_module_names = s([
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
], indices)

controller_names = s([
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
], indices)
