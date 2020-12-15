import collections

#IMU_TOPIC = "/imu/data"
IMU_TOPIC = "/imu/data"
RC_OVERRIDE_TOPIC = "/mavros/rc/override"
GOAL_POSE_TOPIC = "/goal_pose"
AR_MARKER_TOPIC = "/bluerov_controller/ar_tag_detector"

MIN_WRIST_PWM = 1200
MAX_WRIST_PWM = 1520

# the angle it starts at
GRIPPER_ROTATION_INITIAL = 1500

BuildStep = collections.namedtuple('BuildStep', ['pickup_slot', 'drop_slot', 'pickup_wrist_pwm', 'drop_wrist_pwm'])

# so what does the implementation look like? Another assembly action with its termination criterion. Another tracker subclass. Need to integrate it by switching the active controller.

# staggered 8-block build
#BUILD_PLAN = [
#    BuildStep(pickup_slot=(2,0), drop_slot=(0,10)),
#    BuildStep(pickup_slot=(2,4), drop_slot=(0,5)),
#    BuildStep(pickup_slot=(1,0), drop_slot=(0,0)),
#    BuildStep(pickup_slot=(1,4), drop_slot=(1,11)),
#    BuildStep(pickup_slot=(1,8), drop_slot=(1,6)),
#    BuildStep(pickup_slot=(0,0), drop_slot=(1,1)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(2,6)),
#    BuildStep(pickup_slot=(0,8), drop_slot=(2,1)),
#]

# 6 block pyramid
#BUILD_PLAN = [
#    BuildStep(pickup_slot=(2,0), drop_slot=(0,10)),
#    BuildStep(pickup_slot=(2,4), drop_slot=(0,5)),
#    BuildStep(pickup_slot=(1,0), drop_slot=(0,0)),
#    BuildStep(pickup_slot=(1,4), drop_slot=(1,7)),
#    BuildStep(pickup_slot=(1,8), drop_slot=(1,2)),
#    BuildStep(pickup_slot=(0,0), drop_slot=(2,4)),
#]

# 10 block repeatability 
#BUILD_PLAN = [
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#    BuildStep(pickup_slot=(0,4), drop_slot=(0,4)),
#]

PICKUP_PLATFORM_DIMENSIONS = [4, 12]
DROP_PLATFORM_DIMENSIONS = [4, 15]

MIN_PICKUP_SLOT = [-1.94, 0.42, -0.35, 0, 0, 0]
MIN_DROP_SLOT = [-1.94, -0.115, -0.35, 0, 0, 0]

TRACKED_MARKER_ID = 5
TIGHT_POSE_TOLERANCE = [0.025, 0.025, 0.035, float("inf"), float("inf"), 0.018]
COARSE_POSE_TOLERANCE = [0.04, 0.04, 0.04, float("inf"), float("inf"), 0.05]
ULTRA_COARSE_POSE_TOLERANCE = [0.10, 0.10, 0.10, float("inf"), float("inf"), 0.08]

BINARY_P_POSE_TOLERANCE = [0.2, 0.2, 0.2, float("inf"), float("inf"), 0.10]

MAIN_LOOP_RATE = 40

# for deep end
#CENTER_BACK_POSE =  [-2.20, 0.15, 0.28, 0.0, 0, 0]

# for shallow end
CENTER_BACK_POSE =  [-1.30, 0.00, 0.12, 0.0, 0, 0]

SLOT_X_STRIDE = 0.083
SLOT_Z_STRIDE = 0.19

EXPERIMENT_MAX_DURATION_SECONDS = 300.0

BLOCK_HELD_Z_I_GAIN = 0.15
BLOCK_HELD_X_I_GAIN = 0.15
BLOCK_HELD_Y_I_GAIN = 0.15

DEFAULT_X_I_GAIN = 0.10
DEFAULT_Y_I_GAIN = 0.10
DEFAULT_Z_I_GAIN = 0.10

MID_LOW_Z_OFFSET = 0.12
HIGH_Z_OFFSET = 0.15

SLOW_MOVE_HOLD_TIME = 6.0
RAPID_MOVE_HOLD_TIME = 3.0
GRIPPER_HOLD_TIME = 2.5

INTERMEDIATE_WAYPOINT_DISTANCE = 0.25

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
