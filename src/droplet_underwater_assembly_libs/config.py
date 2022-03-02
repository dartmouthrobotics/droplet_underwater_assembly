import collections

#IMU_TOPIC = "/imu/data"
IMU_TOPIC = "/mini_ahrs_ros/imu"
RC_OVERRIDE_TOPIC = "/mavros/rc/override"
DEPTH_TOPIC = "/mavros/imu/static_pressure"
GOAL_POSE_TOPIC = "/goal_pose"
AR_MARKER_TOPIC = "/bluerov_controller/ar_tag_detector"
BUILD_PHASE_TOPIC = "/droplet_underwater_assembly/build_phase"

MIN_SERVO_PWM = 1000
MAX_SERVO_PWM = 2100

# the angle it starts at
WRIST_ROTATION_INITIAL = 1440
FINGER_POSITION_INITIAL = 1400
FINGER_OPEN_POSITION = 2100
FINGER_CLOSED_POSITION = 1500

DEFAULT_POSITION_HOLD_TIME = 6.0

#WRIST_SERVO_INDEX = 1
FINGER_SERVO_INDEX = 2

MAV_CMD_DO_SET_SERVO = 183

MARKER_LOSS_TIME = 10.0

SERVO_TICK_DELAY = 0.05

RANDOMLY_DISPLACE_CENTER_BACK = True
RANDOM_DISPLACEMENT_RANGE = [0.2, 0.2, 0.1, 0.2]

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

MIN_PICKUP_SLOT = [-1.94, 0.12, -0.32, 0, 0, 0]
MIN_DROP_SLOT = [-1.94, -0.115, -0.35, 0, 0, 0]

TRACKED_MARKER_ID = 0
TIGHT_POSE_TOLERANCE = [0.02, 0.02, 0.025, float("inf"), float("inf"), 0.02]
COARSE_POSE_TOLERANCE = [0.06, 0.06, 0.06, float("inf"), float("inf"), 0.05]
ULTRA_COARSE_POSE_TOLERANCE = [0.10, 0.10, 0.10, float("inf"), float("inf"), 0.18]

BINARY_P_POSE_TOLERANCE = [0.2, 0.2, 0.2, float("inf"), float("inf"), 0.10]

MAIN_LOOP_RATE = 60

# for deep end
#CENTER_BACK_POSE =  [-2.20, 0.15, 0.28, 0.0, 0, 0]

# for shallow end
CENTER_BACK_POSE =  [-2.55, 0.40, 0.05, 0.0, 0, 0]
CENTER_BACK_POSE_RIGHT =  [-2.55, -0.40, 0.05, 0.0, 0, 0]

SLOT_X_STRIDE = 0.083
SLOT_Z_STRIDE = 0.19

EXPERIMENT_MAX_DURATION_SECONDS = 7200.0

BLOCK_HELD_Z_I_GAIN = 0.15
BLOCK_HELD_X_I_GAIN = 0.10
BLOCK_HELD_Y_I_GAIN = 0.10

DEFAULT_X_I_GAIN = 0.10
DEFAULT_Y_I_GAIN = 0.10
DEFAULT_Z_I_GAIN = 0.10

MID_LOW_Z_OFFSET = 0.07
HIGH_Z_OFFSET = 0.25

SLOW_MOVE_HOLD_TIME = 6.0
RAPID_MOVE_HOLD_TIME = 3.0
GRIPPER_HOLD_TIME = 0.0

INTERMEDIATE_WAYPOINT_DISTANCE = 0.10

# BALLAST PARAMS
BALLAST_SCUBA_SERVO_INDEX = 1
BALLAST_TANK_OUTLET_SERVO_INDEX = 0
BALLAST_SCUBA_VALVE_FULL_OPEN_PWM = 1615 #1776
BALLAST_TANK_OUTLET_FULL_OPEN_PWM = 1675
BALLAST_TANK_OUTLET_CLOSED_PWM = 1540
BALLAST_SCUBA_VALVE_CLOSED_PWM = 1540
BALLAST_AIR_IN_PULSE_TIME = 0.25
BALLAST_AIR_OUT_PULSE_TIME = 1.0
BALLAST_AIR_OUT_PULSE_OFF_TIME = 3.0
BALLAST_AIR_IN_PULSE_OFF_TIME = 3.0

BALLAST_AIR_FILL_TIME_SECONDS = 1.5
BALLAST_AIR_EMPTY_TIME_SECONDS = 30.0

PRESSURE_TO_MH20 = 0.00010197

GRIPPER_OPEN_TIME = 17.0
GRIPPER_CLOSE_TIME = 17.0
GRIPPER_OPEN_PWM = 1000
GRIPPER_CLOSE_PWM = 2000
GRIPPER_STOP_PWM = 1500
GRIPPER_SERVO_INDEX = 2

MAX_EFFORT_BUOYANCY_CHANGE = 0.90
