joint_ranges = [
    45, # head_tilt
    45, # l_ank_pitch
    45, # l_ank_roll
    45, # l_el
    70, # l_hip_pitch
    45, # l_hip_roll
    20, # l_hip_yaw
    90, # l_knee
    45, # l_sho_pitch
    45, # l_sho_roll
    45, # r_ank_pitch
    45, # r_ank_roll
    45, # r_el
    70, # r_hip_pitch
    45, # r_hip_roll
    20, # r_hip_yaw
    90, # r_knee
    45, # r_sho_pitch
    45, # r_sho_roll
]

position_topics = [
    "/robotis_op3/head_pan_position/command",
    "/robotis_op3/l_ank_pitch_position/command",
    "/robotis_op3/l_ank_roll_position/command",
    "/robotis_op3/l_el_position/command",
    "/robotis_op3/l_hip_pitch_position/command",
    "/robotis_op3/l_hip_roll_position/command",
    "/robotis_op3/l_hip_yaw_position/command",
    "/robotis_op3/l_knee_position/command",
    "/robotis_op3/l_sho_pitch_position/command",
    "/robotis_op3/l_sho_roll_position/command",
    "/robotis_op3/r_ank_pitch_position/command",
    "/robotis_op3/r_ank_roll_position/command",
    "/robotis_op3/r_el_position/command",
    "/robotis_op3/r_hip_pitch_position/command",
    "/robotis_op3/r_hip_roll_position/command",
    "/robotis_op3/r_hip_yaw_position/command",
    "/robotis_op3/r_knee_position/command",
    "/robotis_op3/r_sho_pitch_position/command",
    "/robotis_op3/r_sho_roll_position/command"
]

effort_topics = [
    "/robotis_op3/head_pan_effort/command",
    "/robotis_op3/l_ank_pitch_effort/command",
    "/robotis_op3/l_ank_roll_effort/command",
    "/robotis_op3/l_el_effort/command",
    "/robotis_op3/l_hip_pitch_effort/command",
    "/robotis_op3/l_hip_roll_effort/command",
    "/robotis_op3/l_hip_yaw_effort/command",
    "/robotis_op3/l_knee_effort/command",
    "/robotis_op3/l_sho_pitch_effort/command",
    "/robotis_op3/l_sho_roll_effort/command",
    "/robotis_op3/r_ank_pitch_effort/command",
    "/robotis_op3/r_ank_roll_effort/command",
    "/robotis_op3/r_el_effort/command",
    "/robotis_op3/r_hip_pitch_effort/command",
    "/robotis_op3/r_hip_roll_effort/command",
    "/robotis_op3/r_hip_yaw_effort/command",
    "/robotis_op3/r_knee_effort/command",
    "/robotis_op3/r_sho_pitch_effort/command",
    "/robotis_op3/r_sho_roll_effort/command"
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

links = [
    "robotis_op3::body_link",
    "robotis_op3::head_pan_link",
    "robotis_op3::head_tilt_link",
    "robotis_op3::l_hip_yaw_link",
    "robotis_op3::l_hip_roll_link",
    "robotis_op3::l_hip_pitch_link",
    "robotis_op3::l_knee_link",
    "robotis_op3::l_ank_pitch_link",
    "robotis_op3::l_ank_roll_link",
    "robotis_op3::l_sho_pitch_link",
    "robotis_op3::l_sho_roll_link",
    "robotis_op3::l_el_link",
    "robotis_op3::r_hip_yaw_link",
    "robotis_op3::r_hip_roll_link",
    "robotis_op3::r_hip_pitch_link",
    "robotis_op3::r_knee_link",
    "robotis_op3::r_ank_pitch_link",
    "robotis_op3::r_ank_roll_link",
    "robotis_op3::r_sho_pitch_link",
    "robotis_op3::r_sho_roll_link",
    "robotis_op3::r_el_link",
]