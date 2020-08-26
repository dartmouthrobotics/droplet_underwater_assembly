TRACKED_MARKER_ID = 0
TIGHT_POSE_TOLERANCE = [0.005, 0.005, float("inf"), float("inf"), float("inf"), 0.008]
COARSE_POSE_TOLERANCE = [0.04, 0.04, float("inf"), float("inf"), float("inf"), 0.05]

OVER_BLOCK_1_POSE_LOW = [-1.36, -0.140, -0.07, 0, 0, 0] # for before grabbing
OVER_BLOCK_1_POSE_HIGH = [-1.36, -0.140, -0.00, 0, 0, 0] # for after grabbing
CENTER_BACK_POSE =  [-1.65, -0.133, 0.02, 0, 0, 0]

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
