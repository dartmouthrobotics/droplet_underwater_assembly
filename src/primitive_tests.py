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
import utils
import rospkg
import reporting

from mavros_msgs.srv import CommandBool
import tf

import geometry_msgs.msg
import os

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
GOAL_POSE_XYZRPY = [-1.0, 0, 0, 0, 0, 0.0]
LATEST_MARKER_MESSAGE = None
EXPERIMENT_DURATION_SECONDS = 5.0
BINARY_PWM_VALUE = 50
MARKER_MESSAGE_HISTORY = []
POSITION_HISTORY = []
RESPONSE_HISTORY = []
ERROR_HISTORY = []
IMU_HISTORY = []
DRY_RUN = True
TIMES_TRACKED_MARKER_SEEN = 0

debug_pose_publisher = None
latest_imu_message = None


class VehicleConfig:
    def __init__(self):
        self.yaw_motors = [6]
        self.yaw_directions = [1]

        self.y_motors = [4, 7]
        self.y_directions = [-1, 1]

        self.x_motors = [1, 4]

        self.primitives = {
            "+X":   [1500, 1550, 1500, 1500, 1550, 1500, 1500, 1500], # forward
            "NULL": [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500], # nothing
        }

        for val in self.primitives.values:
            assert(len(val) == 8)

            for speed in val:
                assert(speed >= 1400)
                assert(speed <= 1600)

        assert(len(self.yaw_motors) == len(self.yaw_directions))
        assert(len(self.y_motors) == len(self.y_directions))


VEHICLE_CONFIG = VehicleConfig()


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


def get_next_rc_message(pose_error):
    yaw_error = pose_error[5]
    y_error = pose_error[1]

    rc_message = utils.construct_stop_rc_message()

    primitive = "+X"

    for (motor_number, motor_speed) in enumerate(VEHICLE_CONFIG.primitives[primitive]):
        rc_message.channels[motor_number] = motor_speed

    return rc_message


# first lets do a dry run. What does a dry run look like?
# well first first I need to figure out the right motors
def run_binary_P_control_experiment(rc_override_publisher, debug_pose_publisher):
    global POSITION_HISTORY, RESPONSE_HISTORY, ERROR_HISTORY, latest_imu_message, IMU_HISTORY

    publish_rate = rospy.Rate(40)
    stop_message = utils.construct_stop_rc_message()

    if abs(BINARY_PWM_VALUE) > 100:
        rospy.logerr("Too fast!!!!! PWM must be less than 60. You don't want to make a fountain in the lab do you?")
        sys.exit(1)

    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)

    if not DRY_RUN:
        utils.set_motor_arming(True)

    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)

    start_time = datetime.datetime.now()
    rospy.loginfo("Running control loop with binary pwm {}...".format(BINARY_PWM_VALUE))

    while ((datetime.datetime.now() - start_time).total_seconds() < float(EXPERIMENT_DURATION_SECONDS)) and not rospy.is_shutdown():
        robot_pose = utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE, debug_pose_publisher)
        robot_pose_xyzrpy = utils.to_xyzrpy(*robot_pose)

        pose_error = utils.get_error(robot_pose_xyzrpy, GOAL_POSE_XYZRPY)

        go_message = get_next_rc_message(pose_error)

        POSITION_HISTORY.append(robot_pose_xyzrpy)
        RESPONSE_HISTORY.append(go_message)
        ERROR_HISTORY.append(pose_error)
        IMU_HISTORY.append(latest_imu_message)

        if any([abs(1500 - channel) > 200 for channel in go_message.channels]):
            rospy.loginfo("Too much pwm! Safety stopping on message: {}".format(go_message))

            if not DRY_RUN:
                set_motor_arming(False)

            sys.exit(1)

        rc_override_publisher.publish(go_message)
        publish_rate.sleep()

    rospy.loginfo("Finished data collection")
    if not DRY_RUN:
        utils.set_motor_arming(False)

    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)


def wait_for_marker_data():
    poll_rate = rospy.Rate(10)
    while TIMES_TRACKED_MARKER_SEEN < 100:
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

    rospy.loginfo("Running binary PID control experiment with pwm {pwm} and time {time}".format(
        pwm=BINARY_PWM_VALUE,
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
        vehicle_config=VEHICLE_CONFIG,
        position_history=POSITION_HISTORY,
        response_history=RESPONSE_HISTORY,
        error_history=ERROR_HISTORY,
        goal_position=GOAL_POSE_XYZRPY,
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
