import trajectory_tracker
import collections

TRACKED_MARKER_ID = 0
TIGHT_POSE_TOLERANCE = [0.025, 0.025, 0.025, float("inf"), float("inf"), 0.018]
COARSE_POSE_TOLERANCE = [0.04, 0.04, 0.04, float("inf"), float("inf"), 0.05]

MAIN_LOOP_RATE = 40

# tag to ground: 96cm
# base link to gripper center: 68cm
# grip height: 12cm

CENTER_BACK_POSE =  [-1.89, 0.2, 0.08, -0.35, 0, 0]

SLOT_X_STRIDE = 0.08
SLOT_Z_STRIDE = 0.18

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


EXPERIMENT_DURATION_SECONDS = 1200.0

PLATFORM_FRAME_ID = "/build_platform_0"


BLOCK_HELD_Z_I_GAIN = 0.15
BLOCK_HELD_X_I_GAIN = 0.1
BLOCK_HELD_Y_I_GAIN = 0.1

DEFAULT_X_I_GAIN = 0.05
DEFAULT_Y_I_GAIN = 0.05
DEFAULT_Z_I_GAIN = 0.05
