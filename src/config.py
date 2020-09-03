import trajectory_tracker
import collections

TRACKED_MARKER_ID = 0
TIGHT_POSE_TOLERANCE = [0.025, 0.025, float("inf"), float("inf"), 0.02, 0.018]
COARSE_POSE_TOLERANCE = [0.04, 0.04, float("inf"), float("inf"), 0.04, 0.05]

MAIN_LOOP_RATE = 40

# tag to ground: 96cm
# base link to gripper center: 68cm
# grip height: 12cm

OVER_BLOCK_1_POSE_LOW = [-1.44, -0.14, -0.20, 0, 0, 0] # for before grabbing
OVER_BLOCK_1_POSE_HIGH = [-1.44, -0.14, 0.12, 0, 0, 0] # for after grabbing
CENTER_BACK_POSE =  [-1.85, -0.17, 0.09, 0, 0, 0]

EXPERIMENT_DURATION_SECONDS = 1200.0

# deprecated 
STOP_TIME_SECONDS = 0.10
GO_TIME_SECONDS = 0.3
MOTION_PRIMITIVES = [
    trajectory_tracker.MotionPrimitive("+PITCH",    [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("+ROLL",    [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("+Z",    [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),

    trajectory_tracker.MotionPrimitive("NULL",      [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("+Yaw",      [1500, 1539, 1500, 1500, 1500, 1500, 1516, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("-Yaw",      [1500, 1465, 1500, 1500, 1500, 1500, 1500, 1465], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("ONE_MOTOR", [1500, 1445, 1500, 1500, 1590, 1500, 1535, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("-Y",        [1500, 1500, 1500, 1500, 1455, 1500, 1456, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("+Y",        [1500, 1500, 1500, 1500, 1545, 1500, 1535, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("+X",        [1500, 1445, 1500, 1500, 1545, 1500, 1500, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
    trajectory_tracker.MotionPrimitive("-X",        [1500, 1545, 1500, 1500, 1445, 1500, 1500, 1500], None, GO_TIME_SECONDS, STOP_TIME_SECONDS),
]

PLATFORM_FRAME_ID = "/build_platform_0"

PlatformSlot = collections.namedtuple("PlatformSlot", ["frame_id", "location"])
PLATFORM_SLOTS = [
    PlatformSlot("slot_0", [-1.44, -0.12, -0.6, 0.0, 0.0, 0.0])
]
