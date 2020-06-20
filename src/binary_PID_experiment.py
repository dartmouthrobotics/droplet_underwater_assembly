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
GOAL_POSE_XYZRPY = [-0.75, -0.1, 0, 0, 0, 0.0]
LATEST_MARKER_MESSAGE = None
EXPERIMENT_DURATION_SECONDS = 100.0
BINARY_PWM_VALUE = 50
MARKER_MESSAGE_HISTORY = []
POSITION_HISTORY = []
RESPONSE_HISTORY = []
ERROR_HISTORY = []
IMU_HISTORY = []
GOAL_POSE_HISTORY = []
DRY_RUN = True
TIMES_TRACKED_MARKER_SEEN = 0
GRIPPER_HANDLER = gripper_handler.GripperHandler()

OVER_BLOCK_1_POSE = [-0.73, -0.1, 0, 0, 0, 0]
CENTER_BACK_POSE =  [-0.83, -0.1, 0, 0, 0, 0]
POSE_TOLERANCE = [0.03, 0.03, float("inf"), float("inf"), float("inf"), 0.05]

debug_pose_publisher = None
latest_imu_message = None

MOTION_PRIMITIVES = [
    trajectory_tracker.MotionPrimitive("+X", [1500, 1445, 1500, 1500, 1545, 1500, 1430, 1430], None),
    trajectory_tracker.MotionPrimitive("-X", [1500, 1545, 1500, 1500, 1445, 1500, 1500, 1500], None),
    trajectory_tracker.MotionPrimitive("+Y", [1500, 1540, 1500, 1500, 1540, 1500, 1545, 1455], None),
    trajectory_tracker.MotionPrimitive("-Y", [1500, 1455, 1500, 1500, 1455, 1500, 1455, 1540], None),
    trajectory_tracker.MotionPrimitive("-Yaw", [1500, 1445, 1500, 1500, 1500, 1500, 1500, 1445], None),
    trajectory_tracker.MotionPrimitive("+Yaw", [1500, 1525, 1500, 1500, 1500, 1500, 1500, 1525], None),
    trajectory_tracker.MotionPrimitive("NULL", [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500], None),
    trajectory_tracker.MotionPrimitive("ONE_MOTOR", [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1430], None),
]

TRAJECTORY_TRACKER = trajectory_tracker.BinaryTrajectoryTracker(MOTION_PRIMITIVES)
#TRAJECTORY_TRACKER = trajectory_tracker.SinglePrimitiveTracker(MOTION_PRIMITIVES)
TRAJECTORY_TRACKER.set_goal_position(GOAL_POSE_XYZRPY)

class AssemblyAction(object):
    def __init__(self, action_type, goal_pose):
        self.valid_types = ['move', 'open_gripper', 'close_gripper']
        self.action_type = action_type
        self.goal_pose = goal_pose
        self.start_time = None
        self.reached_goal_time = None
        self.position_hold_time = 6.0
        self.gripper_hold_time = 2.0

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
                reached_goal = all([abs(error) < tolerance for (error, tolerance) in zip(pose_error, POSE_TOLERANCE)])

                if reached_goal:
                    self.reached_goal_time = rospy.Time.now()

                return False
            else:
                return (rospy.Time.now() - self.reached_goal_time).to_sec() > self.position_hold_time

        if self.action_type == 'open_gripper' or self.action_type == 'close_gripper':
            return (rospy.Time.now() - self.start_time).to_sec() > GRIPPER_HANDLER.toggle_time_seconds + self.gripper_hold_time

        raise Exception("Unrecognized action type!")


ACTIONS = [
    AssemblyAction('move', OVER_BLOCK_1_POSE),
    AssemblyAction('close_gripper', OVER_BLOCK_1_POSE),
    AssemblyAction('move', CENTER_BACK_POSE),
    #AssemblyAction('move', OVER_BLOCK_1_POSE),
    #AssemblyAction('open_gripper', OVER_BLOCK_1_POSE),
    #AssemblyAction('move', CENTER_BACK_POSE),
]


def imu_callback(imu_message):
    global latest_imu_message
    latest_imu_message = imu_message


def marker_callback(marker_message):
    global LATEST_MARKER_MESSAGE, TIMES_TRACKED_MARKER_SEEN
    MARKER_MESSAGE_HISTORY.append(marker_message)

    for marker in marker_message.markers:
        if marker.id == TRACKED_MARKER_ID:
            LATEST_MARKER_MESSAGE = marker
            TIMES_TRACKED_MARKER_SEEN = TIMES_TRACKED_MARKER_SEEN + 1


def run_binary_P_control_experiment(rc_override_publisher, debug_pose_publisher):
    global POSITION_HISTORY, RESPONSE_HISTORY, ERROR_HISTORY, latest_imu_message, IMU_HISTORY, GOAL_POSE_XYZRPY, GOAL_POSE_HISTORY

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

    while ((datetime.datetime.now() - start_time).total_seconds() < float(EXPERIMENT_DURATION_SECONDS)) and not rospy.is_shutdown():
        goal_not_reached = current_action.reached_goal_time is None

        GOAL_POSE_XYZRPY = current_action.goal_pose
        TRAJECTORY_TRACKER.set_goal_position(GOAL_POSE_XYZRPY)

        robot_pose = utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE, debug_pose_publisher)
        robot_pose_xyzrpy = utils.to_xyzrpy(*robot_pose)

        TRAJECTORY_TRACKER.set_current_position(robot_pose_xyzrpy)
        pose_error = TRAJECTORY_TRACKER.get_error()

        # act on the current action if it is not complete
        if not current_action.is_started:
            if current_action.action_type == 'open_gripper':
                GRIPPER_HANDLER.start_opening()
            elif current_action.action_type == 'close_gripper':
                GRIPPER_HANDLER.start_closing()

            print("Starting action: {}".format(current_action))
            current_action.start()
        else:
            if current_action.is_complete(pose_error):
                rospy.loginfo("Completed action: {}".format(current_action))
                current_action = ACTIONS.pop(0)

        if goal_not_reached and current_action.reached_goal_time is not None:
            rospy.loginfo("Reached goal position. Holding.")

        go_message = TRAJECTORY_TRACKER.get_next_motion_primitive().as_rc_override

        GRIPPER_HANDLER.update()
        GRIPPER_HANDLER.mix_into_rc_override_message(go_message)

        if current_action.action_type == 'move':
            assert(go_message.channels[GRIPPER_HANDLER.channel] == 1500)

        GOAL_POSE_HISTORY.append(GOAL_POSE_XYZRPY)
        POSITION_HISTORY.append(robot_pose_xyzrpy)
        RESPONSE_HISTORY.append(go_message)
        ERROR_HISTORY.append(pose_error)
        IMU_HISTORY.append(latest_imu_message)

        if any([abs(1500 - channel) > 150 for channel in go_message.channels]):
            rospy.logwarn("Too much pwm! Safety stopping on message: {}".format(go_message))

            if not DRY_RUN:
                set_motor_arming(False)

            sys.exit(1)

        rc_override_publisher.publish(go_message)
        publish_rate.sleep()

    rospy.loginfo("Opening gripper after run")

    rospy.loginfo("Finished data collection")
    if not DRY_RUN:
        utils.set_motor_arming(False)

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
        marker_callback
    )

    imu_subscriber = rospy.Subscriber(
        "/mavros/imu/data_raw",
        sensor_msgs.msg.Imu,
        imu_callback
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
        report_output=report_output
    )

    rospy.loginfo("Finished outputting report.")

if __name__ == "__main__":
    main()
