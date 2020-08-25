#! /usr/bin/python

import rospy
import datetime
import sys
import json
import numpy
import math
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet
import datetime
import mavros_msgs.msg
import stag_ros.msg
import sensor_msgs.msg
import rospkg
import reporting
import trajectory_tracker
from mavros_msgs.srv import CommandBool
import tf
import geometry_msgs.msg
import os
import collections

import utils
import gripper_handler
"""
Motor locations:

Top refers to motors facing up on top of bluerov
Bottom (the ones facing inward on bottom)
Front means towards the camera
Left is to the left if you are looking at the back of the bluerov towards the camera (ie you are behind it)

1: top back right
2: bottom front right
3: top front right
4: top front left
5: bottom front left
6: top back left
7: bottom back left
8: bottom back right
"""


TRACKED_MARKER_ID = 0
LATEST_MARKER_MESSAGE = None
EXPERIMENT_DURATION_SECONDS = 240.0
MARKER_MESSAGE_HISTORY = []
POSITION_HISTORY = []
RESPONSE_HISTORY = []
ERROR_HISTORY = []
IMU_HISTORY = []
GOAL_POSE_HISTORY = []
RAW_VELOCITY_HISTORY = []
RUNNING_EXPERIMENT = False
VELOCITY_HISTORY = []
DRY_RUN = True
TIMES_TRACKED_MARKER_SEEN = 0
GRIPPER_HANDLER = gripper_handler.GripperHandler()
PRIMITIVE_HISTORY = []

OVER_BLOCK_1_POSE_LOW = [-1.36, -0.140, -0.07, 0, 0, 0] # for before grabbing
OVER_BLOCK_1_POSE_HIGH = [-1.36, -0.140, -0.00, 0, 0, 0] # for after grabbing
CENTER_BACK_POSE =  [-1.65, -0.133, 0.02, 0, 0, 0]

TIGHT_POSE_TOLERANCE = [0.005, 0.005, float("inf"), float("inf"), float("inf"), 0.008]
COARSE_POSE_TOLERANCE = [0.04, 0.04, float("inf"), float("inf"), float("inf"), 0.05]

debug_pose_publisher = None
latest_imu_message = None

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

#TRAJECTORY_TRACKER = trajectory_tracker.BinaryTrajectoryTracker(MOTION_PRIMITIVES)
#TRAJECTORY_TRACKER = trajectory_tracker.PredictivePositionTracker(MOTION_PRIMITIVES, 0.5)
#TRAJECTORY_TRACKER = trajectory_tracker.SinglePrimitiveTracker(MOTION_PRIMITIVES)
TRAJECTORY_TRACKER = trajectory_tracker.PIDTracker(
    x_p=3.05,
    y_p=3.05,
    yaw_p=1.95, 
    x_d=-2.0, 
    y_d=-0.25,
    yaw_d=1.0,
    x_i=0.0,
    y_i=0.0,
    yaw_i=0.0,
    roll_p=-0.5,
    roll_i=0.0,
    roll_d=1.0,
    z_p=2.0,
    z_i=0.0,
    z_d=-1.0,
    pitch_p=-1.5,
    pitch_i=0.0,
    pitch_d=1.0
)

#TRAJECTORY_TRACKER = trajectory_tracker.PIDTracker(
#    x_p=0.00,
#    y_p=0.00,
#    yaw_p=0.00, 
#    x_d=0.0, 
#    y_d=-0.00,
#    yaw_d=0.0,
#    x_i=0.0,
#    y_i=0.0,
#    yaw_i=0.0,
#    roll_p=-0.5,
#    roll_i=0.0,
#    roll_d=1.0,
#    z_p=2.0,
#    z_i=0.0,
#    z_d=-1.0,
#    pitch_p=-1.5,
#    pitch_i=0.0,
#    pitch_d=1.0
#)

class AssemblyAction(object):
    def __init__(self, action_type, goal_pose, pose_tolerance):
        self.valid_types = ['move', 'open_gripper', 'close_gripper']
        self.action_type = action_type
        self.goal_pose = goal_pose
        self.start_time = None
        self.reached_goal_time = None
        self.position_hold_time = 2.0
        self.gripper_hold_time = 2.0
        self.pose_tolerance = pose_tolerance

        assert(self.action_type in self.valid_types)
        assert(len(self.goal_pose) == 6)

    def __str__(self):
        if self.action_type == "open_gripper" or self.action_type == "close_gripper":

            return "Action type: {}".format(self.action_type)

        return "Move to: {}".format(self.goal_pose)

    def start(self):
        self.start_time = rospy.Time.now()

    @property
    def is_started(self):
        return self.start_time is not None

    def is_complete(self, pose_error):
        if self.start_time is None:
            rospy.logerr("Cannot complete an action that hasn't been started.")
            return False

        if self.action_type == 'move':
            if self.reached_goal_time is None:
                reached_goal = all([abs(error) < tolerance for (error, tolerance) in zip(pose_error, self.pose_tolerance)])

                if reached_goal:
                    print(pose_error)
                    self.reached_goal_time = rospy.Time.now()

                return False
            else:
                return (rospy.Time.now() - self.reached_goal_time).to_sec() > self.position_hold_time

        if self.action_type == 'open_gripper' or self.action_type == 'close_gripper':
            return (rospy.Time.now() - self.start_time).to_sec() > GRIPPER_HANDLER.toggle_time_seconds + self.gripper_hold_time

        raise Exception("Unrecognized action type!")


ACTIONS = [
    # round 1
    AssemblyAction('move', OVER_BLOCK_1_POSE_LOW, TIGHT_POSE_TOLERANCE),
    AssemblyAction('close_gripper', OVER_BLOCK_1_POSE_LOW, COARSE_POSE_TOLERANCE),
    AssemblyAction('move', OVER_BLOCK_1_POSE_HIGH, COARSE_POSE_TOLERANCE),
    AssemblyAction('move', CENTER_BACK_POSE, COARSE_POSE_TOLERANCE),
    AssemblyAction('move', OVER_BLOCK_1_POSE_HIGH, TIGHT_POSE_TOLERANCE),
    AssemblyAction('open_gripper', OVER_BLOCK_1_POSE_HIGH, TIGHT_POSE_TOLERANCE),
    AssemblyAction('move', CENTER_BACK_POSE, COARSE_POSE_TOLERANCE),
]

#ACTIONS = [
#    # round 1
#    AssemblyAction('move', OVER_BLOCK_1_POSE),
#]


def average_velocity(values_history, number_terms):
    total = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if number_terms > len(values_history):
        number_terms = len(values_history)
    
    for i in range(number_terms):
        for j in range(6):
            total[j] = total[j] + values_history[len(values_history) - 1 - i][j]

    for j in range(6):
        total[j] = total[j] / float(number_terms)

    return total


def imu_callback(imu_message):
    global latest_imu_message
    latest_imu_message = imu_message

    TRAJECTORY_TRACKER.set_latest_imu_reading(latest_imu_message)


def marker_callback(marker_message):
    global LATEST_MARKER_MESSAGE, TIMES_TRACKED_MARKER_SEEN, LATEST_VELOCITY, RAW_VELOCITY_HISTORY
    MARKER_MESSAGE_HISTORY.append(marker_message)

    if LATEST_MARKER_MESSAGE is not None:
        last_marker_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE))

    for marker in marker_message.markers:
        if marker.id == TRACKED_MARKER_ID:
            if LATEST_MARKER_MESSAGE is not None:
                current_marker_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(marker))
                distance_travelled = utils.get_error(last_marker_pose, current_marker_pose)
                time_delta_seconds = (marker.header.stamp - LATEST_MARKER_MESSAGE.header.stamp).to_sec()
                LATEST_VELOCITY = [value / time_delta_seconds for value in distance_travelled]

            LATEST_MARKER_MESSAGE = marker
            TIMES_TRACKED_MARKER_SEEN = TIMES_TRACKED_MARKER_SEEN + 1

            if RUNNING_EXPERIMENT:
                RAW_VELOCITY_HISTORY.append(LATEST_VELOCITY)


def run_binary_P_control_experiment(rc_override_publisher, debug_pose_publisher):
    global POSITION_HISTORY, RESPONSE_HISTORY, ERROR_HISTORY, latest_imu_message, IMU_HISTORY, GOAL_POSE_XYZRPY, GOAL_POSE_HISTORY, PRIMITIVE_HISTORY, VELOCITY_HISTORY, RUNNING_EXPERIMENT

    publish_rate = rospy.Rate(40)
    stop_message = utils.construct_stop_rc_message()

    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)

    if not DRY_RUN:
        utils.set_motor_arming(True)

    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)

    start_time = datetime.datetime.now()

    current_action = ACTIONS.pop(0)

    next_primitive = None
    while ((datetime.datetime.now() - start_time).total_seconds() < float(EXPERIMENT_DURATION_SECONDS)) and not rospy.is_shutdown():
        RUNNING_EXPERIMENT = True
        loop_start = rospy.Time.now()

        goal_not_reached = current_action.reached_goal_time is None

        TRAJECTORY_TRACKER.set_goal_position(current_action.goal_pose)

        VELOCITY_HISTORY.append(LATEST_VELOCITY)
        
        latest_vel_avg = average_velocity(VELOCITY_HISTORY, 2)
        TRAJECTORY_TRACKER.set_current_velocity([latest_vel_avg[0], latest_vel_avg[1], 0.0, 0.0, 0.0, -latest_imu_message.angular_velocity.z])

        robot_pose = utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE, debug_pose_publisher)
        robot_pose_xyzrpy = utils.to_xyzrpy(*robot_pose)

        TRAJECTORY_TRACKER.set_latest_imu_reading(latest_imu_message)
        TRAJECTORY_TRACKER.set_current_position(robot_pose_xyzrpy)
        pose_error = TRAJECTORY_TRACKER.get_error()

        # act on the current action if it is not complete
        if not current_action.is_started:
            tolerance = TIGHT_POSE_TOLERANCE
            reached_goal = all([abs(error) < tol for (error, tol) in zip(pose_error, tolerance)])

            if current_action.action_type == 'open_gripper':
                if reached_goal:
                    GRIPPER_HANDLER.start_opening()
                    PRIMITIVE_HISTORY.append('open_gripper')
            elif current_action.action_type == 'close_gripper':
                if reached_goal:
                    GRIPPER_HANDLER.start_closing()
                    PRIMITIVE_HISTORY.append('close_gripper')


            if current_action.action_type in ['open_gripper', 'close_gripper']:
                if reached_goal:
                    current_action.start()
            else:
                current_action.start()

            if current_action.is_started:
                rospy.loginfo("Starting action: {}".format(current_action))
        else:
            if current_action.is_complete(pose_error):
                rospy.loginfo("Completed action: {}".format(current_action))

                if len(ACTIONS) > 0:
                    current_action = ACTIONS.pop(0)

        if current_action.is_started and current_action.is_complete(pose_error) and len(ACTIONS) == 0:
            rospy.loginfo("Completed all actions! Terminating")
            break

        if goal_not_reached and current_action.reached_goal_time is not None:
            rospy.loginfo("Reached goal position. Holding.")

        next_primitive = TRAJECTORY_TRACKER.get_next_motion_primitive()
        next_primitive.start()
        go_message = TRAJECTORY_TRACKER.get_next_rc_override()

        GRIPPER_HANDLER.update()
        GRIPPER_HANDLER.mix_into_rc_override_message(go_message)

        PRIMITIVE_HISTORY.append(next_primitive.name)

        if current_action.action_type == 'move':
            assert(go_message.channels[GRIPPER_HANDLER.channel] == 1500)

        GOAL_POSE_HISTORY.append(current_action.goal_pose)
        POSITION_HISTORY.append(robot_pose_xyzrpy)
        RESPONSE_HISTORY.append(go_message)
        ERROR_HISTORY.append(pose_error)
        IMU_HISTORY.append(latest_imu_message)

        if any([abs(1500 - channel) > 150 for channel in go_message.channels]):
            rospy.logwarn("Too much pwm! Safety stopping on message: {}".format(go_message))

            if not DRY_RUN:
                set_motor_arming(False)

            sys.exit(1)

        loop_end = rospy.Time.now()

        loop_time = (loop_end - loop_start).to_sec()
        if (loop_time > 0.03):
            rospy.logwarn("Slow loop!")

        rc_override_publisher.publish(go_message)
        publish_rate.sleep()

    rospy.loginfo("Finished data collection")
    RUNNING_EXPERIMENT = False
    if not DRY_RUN:
        try:
            utils.set_motor_arming(False)
        except:
            pass

    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)


def wait_for_marker_data():
    poll_rate = rospy.Rate(10)
    while TIMES_TRACKED_MARKER_SEEN < 100 and not rospy.is_shutdown():
        poll_rate.sleep()


def main():
    global DRY_RUN, debug_pose_publisher
    rospy.init_node("binary_pid_control")

    marker_subscriber = rospy.Subscriber(
        "/bluerov_controller/ar_tag_detector",
        stag_ros.msg.StagMarkers,
        marker_callback,
        queue_size=1
    )

    imu_subscriber = rospy.Subscriber(
        "/imu/data",
        sensor_msgs.msg.Imu,
        imu_callback,
        queue_size=1
    )

    debug_pose_publisher = rospy.Publisher(
        "debug_pose",
        geometry_msgs.msg.PoseStamped,
        queue_size=0
    )

    rc_override_publisher = rospy.Publisher(
        "/mavros/rc/override",
        mavros_msgs.msg.OverrideRCIn,
        queue_size=0
    )

    try:
        DRY_RUN = rospy.get_param("~dry_run")
    except:
        rospy.logwarn("dry_run parameter not provided. Defaulting to test mode.")

    if DRY_RUN:
        rospy.loginfo("Running in test mode.")

    rospy.loginfo("Running control test experiment for {time} seconds".format(
        time=EXPERIMENT_DURATION_SECONDS
    ))

    rospy.loginfo("Waiting for enough marker data....")
    wait_for_marker_data()
    rospy.loginfo("Got marker data, going!!")

    run_binary_P_control_experiment(
        rc_override_publisher,
        debug_pose_publisher
    )

    notes = ""
    try:
        notes = rospy.get_param("~notes") 
    except:
        rospy.logwarn("No notes provided. Section will be blank in report.")

    time_prefix = datetime.datetime.now().strftime("%a_%b_%d_%I_%M_%p")
    rp = rospkg.RosPack()
    output_directory = os.path.join(rp.get_path("minimal_bluerov_control_experiment"), "static_assets", time_prefix)
    os.mkdir(output_directory)

    report_output = "{output_directory}/{time_prefix}-report.txt".format(time_prefix=time_prefix, output_directory=output_directory)

    rospy.loginfo("Outputting report to {}".format(report_output))
    report = reporting.report_results(
        position_history=POSITION_HISTORY,
        response_history=RESPONSE_HISTORY,
        error_history=ERROR_HISTORY,
        goal_pose_history=GOAL_POSE_HISTORY,
        notes=notes,
        output_directory=output_directory,
        data_collection_time=EXPERIMENT_DURATION_SECONDS,
        marker_history=MARKER_MESSAGE_HISTORY,
        imu_history=IMU_HISTORY,
        report_output=report_output,
        primitive_history=PRIMITIVE_HISTORY
    )

    rospy.loginfo("Finished outputting report.")

    velocity_output = "velocities.csv"

    vel_data = "\n".join([
       ",".join(
           ["{0:.3f}".format(axis) for axis in pos]
        ) for pos in RAW_VELOCITY_HISTORY 
    ])

    open(velocity_output, "w").write(vel_data)


if __name__ == "__main__":
    main()
