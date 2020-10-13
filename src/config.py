import collections

# TODO break down large moves

BuildStep = collections.namedtuple('BuildStep', ['pickup_slot', 'drop_slot'])

BUILD_PLAN = [
    BuildStep(pickup_slot=(1,0), drop_slot=(0,10))
]

PICKUP_PLATFORM_DIMENSIONS = [2, 9]
DROP_PLATFORM_DIMENSIONS = [2, 11]

MIN_PICKUP_SLOT = [-1.62, 0.48, -0.35, 0, 0, 0]
MIN_DROP_SLOT = [-1.94, -0.125, -0.35, 0, 0, 0]

TRACKED_MARKER_ID = 0
TIGHT_POSE_TOLERANCE = [0.025, 0.025, 0.025, float("inf"), float("inf"), 0.018]
COARSE_POSE_TOLERANCE = [0.04, 0.04, 0.04, float("inf"), float("inf"), 0.05]

MAIN_LOOP_RATE = 40

CENTER_BACK_POSE =  [-2.10, 0.2, 0.08, 0.0, 0, 0]

SLOT_X_STRIDE = 0.083
SLOT_Z_STRIDE = 0.18

EXPERIMENT_MAX_DURATION_SECONDS = 1200.0

BLOCK_HELD_Z_I_GAIN = 0.15
BLOCK_HELD_X_I_GAIN = 0.1
BLOCK_HELD_Y_I_GAIN = 0.1

DEFAULT_X_I_GAIN = 0.05
DEFAULT_Y_I_GAIN = 0.05
DEFAULT_Z_I_GAIN = 0.05

MID_LOW_Z_OFFSET = 0.12
HIGH_Z_OFFSET = 0.22

SLOW_MOVE_HOLD_TIME = 6.0
RAPID_MOVE_HOLD_TIME = 4.0
GRIPPER_HOLD_TIME = 2.5

INTERMEDIATE_WAYPOINT_DISTANCE = 0.15

#ACTIONS = [
#    # 5 to 1
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_5_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_5_POSE_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('close_gripper', config.OVER_BLOCK_5_POSE_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_5_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_1_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_1_POSE_MID_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('open_gripper', config.OVER_BLOCK_1_POSE_MID_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_1_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#
#    # 4 to 6
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_4_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_4_POSE_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('close_gripper', config.OVER_BLOCK_4_POSE_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_4_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_6_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_6_POSE_MID_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('open_gripper', config.OVER_BLOCK_6_POSE_MID_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_6_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#
#    # 2 to 3
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_2_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_2_POSE_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('close_gripper', config.OVER_BLOCK_2_POSE_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_2_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_3_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_3_POSE_MID_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('open_gripper', config.OVER_BLOCK_3_POSE_MID_LOW, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.OVER_BLOCK_3_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#]

# right bottom back
#OVER_BLOCK_1_POSE_LOW = [-1.15, -0.135, -0.35, 0, 0, 0] # for before grabbing
##OVER_BLOCK_1_POSE_MID_LOW = [-1.15, -0.135, -0.20, 0, 0, 0] # for before grabbing
##OVER_BLOCK_1_POSE_HIGH = [-1.15, -0.135, -0.10, 0, 0, 0] # for after grabbing
#
## left bottom back
#OVER_BLOCK_2_POSE_LOW = [-1.30, 0.46, -0.32, 0, 0, 0] # for before grabbing
##OVER_BLOCK_2_POSE_MID_LOW = [-1.30, 0.46, -0.20, 0, 0, 0] # for before grabbing
##OVER_BLOCK_2_POSE_HIGH = [-1.30, 0.46, -0.10, 0, 0, 0] # for after grabbing
#
## right top
#OVER_BLOCK_3_POSE_LOW = list(OVER_BLOCK_1_POSE_LOW)
##OVER_BLOCK_3_POSE_MID_LOW = list(OVER_BLOCK_1_POSE_MID_LOW)
##OVER_BLOCK_3_POSE_HIGH = list(OVER_BLOCK_1_POSE_HIGH)
## right top
#OVER_BLOCK_3_POSE_LOW[2] = OVER_BLOCK_1_POSE_LOW[2] + 0.18
##OVER_BLOCK_3_POSE_MID_LOW[2] = OVER_BLOCK_1_POSE_MID_LOW[2] + 0.18
##OVER_BLOCK_3_POSE_HIGH[2] = OVER_BLOCK_1_POSE_HIGH[2] + 0.18
#OVER_BLOCK_3_POSE_LOW[0] = OVER_BLOCK_1_POSE_LOW[0] - 0.16
##OVER_BLOCK_3_POSE_MID_LOW[0] = OVER_BLOCK_1_POSE_MID_LOW[0] - 0.16
##OVER_BLOCK_3_POSE_HIGH[0] = OVER_BLOCK_1_POSE_HIGH[0] - 0.16
#
## left top
#OVER_BLOCK_4_POSE_LOW = list(OVER_BLOCK_2_POSE_LOW)
##OVER_BLOCK_4_POSE_MID_LOW = list(OVER_BLOCK_2_POSE_MID_LOW)
##OVER_BLOCK_4_POSE_HIGH = list(OVER_BLOCK_2_POSE_HIGH)
## left top
##OVER_BLOCK_4_POSE_LOW[2] = OVER_BLOCK_2_POSE_LOW[2] + 0.18
##OVER_BLOCK_4_POSE_MID_LOW[2] = OVER_BLOCK_2_POSE_MID_LOW[2] + 0.18
##OVER_BLOCK_4_POSE_HIGH[2] = OVER_BLOCK_2_POSE_HIGH[2] + 0.18
##
### left front
##OVER_BLOCK_5_POSE_LOW = [-1.63, 0.46, -0.32, 0, 0, 0] # for before grabbing
##OVER_BLOCK_5_POSE_MID_LOW = [-1.63, 0.46, -0.20, 0, 0, 0] # for before grabbing
##OVER_BLOCK_5_POSE_HIGH = [-1.63, 0.46, -0.10, 0, 0, 0] # for after grabbing
##
### right bottom front
##OVER_BLOCK_6_POSE_LOW = [-1.56, -0.135, -0.35, 0, 0, 0] # for before grabbing
##OVER_BLOCK_6_POSE_MID_LOW = [-1.56, -0.135, -0.20, 0, 0, 0] # for before grabbing
##OVER_BLOCK_6_POSE_HIGH = [-1.56, -0.135, -0.10, 0, 0, 0] # for after grabbing
