#! /usr/bin/python
import sys
import datetime
import os

import rospy
import rospkg
from mavros_msgs.srv import CommandBool
import tf

import geometry_msgs.msg
import mavros_msgs.msg
import stag_ros.msg
import sensor_msgs.msg

import utils
import gripper_handler
import trajectory_tracker
import assembly_action
import config

import build_platform


"""
TODO:
    * Handling of non-level build platforms
    Handling (more gracefully) of more slots on the platform
"""

"""
To handle a non-level build platform, what can we start with? Lets get a bag file (after fixing the hand, maybe on Sunday.)
The way that the non-level platform would work is that we compute the platform's roll and pitch in world frame based on the imu's orientation and each tag reading.

How can we do this? I guess a running average would be the best since it should never change.
How does it influence the things we tell the robot to do?
"""


LATEST_MARKER_MESSAGE = None
RAW_VELOCITY_HISTORY = []
RUNNING_EXPERIMENT = False
VELOCITY_HISTORY = []
DRY_RUN = True
TIMES_TRACKED_MARKER_SEEN = 0
GRIPPER_HANDLER = gripper_handler.GripperHandler()

latest_imu_message = None
goal_pose_publisher = None
transform_broadcaster = None

TRAJECTORY_TRACKER = trajectory_tracker.PIDTracker(
    x_p=4.00,
    y_p=4.00,
    yaw_p=2.35, 
    x_d=-1.0, 
    y_d=-0.25,
    yaw_d=1.0,
    x_i=0.0,
    y_i=0.0,
    yaw_i=0.0,
    roll_p=-0.3,
    roll_i=0.0,
    roll_d=1.0,
    z_p=3.5,
    z_i=0.0,
    z_d=-0.50,
    pitch_p=-1.0,
    pitch_i=0.0,
    pitch_d=0.75,
)

BUILD_PLATFORM = build_platform.BuildPlatform(
    pickup_dimensions=config.PICKUP_PLATFORM_DIMENSIONS,
    drop_dimensions=config.DROP_PLATFORM_DIMENSIONS,
    row_spacing=config.SLOT_Z_STRIDE,
    column_spacing=config.SLOT_X_STRIDE,
    min_pickup_slot=config.MIN_PICKUP_SLOT,
    min_drop_slot=config.MIN_DROP_SLOT,
    frame_id="/build_platform"
)

ACTIONS = BUILD_PLATFORM.confvert_build_steps_into_assembly_actions(config.BUILD_PLAN, GRIPPER_HANDLER)

def imu_callback(imu_message):
    global latest_imu_message
    latest_imu_message = imu_message

    TRAJECTORY_TRACKER.set_latest_imu_reading(latest_imu_message)


def marker_callback(marker_message):
    global LATEST_MARKER_MESSAGE, TIMES_TRACKED_MARKER_SEEN, LATEST_VELOCITY, RAW_VELOCITY_HISTORY

    if LATEST_MARKER_MESSAGE is not None:
        last_marker_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE))

    for marker in marker_message.markers:
        if marker.id == config.TRACKED_MARKER_ID:
            if LATEST_MARKER_MESSAGE is not None:
                current_marker_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(marker))
                distance_travelled = utils.get_error(last_marker_pose, current_marker_pose)
                time_delta_seconds = (marker.header.stamp - LATEST_MARKER_MESSAGE.header.stamp).to_sec()
                LATEST_VELOCITY = [value / time_delta_seconds for value in distance_travelled]

            LATEST_MARKER_MESSAGE = marker
            TIMES_TRACKED_MARKER_SEEN = TIMES_TRACKED_MARKER_SEEN + 1

            if RUNNING_EXPERIMENT:
                RAW_VELOCITY_HISTORY.append(LATEST_VELOCITY)


def run_build_plan(rc_override_publisher):
    global latest_imu_message, VELOCITY_HISTORY, RUNNING_EXPERIMENT

    publish_rate = rospy.Rate(config.MAIN_LOOP_RATE)
    stop_message = utils.construct_stop_rc_message()

    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)

    if not DRY_RUN:
        utils.set_motor_arming(True)
        rospy.loginfo("Motors armed.")

    start_time = datetime.datetime.now()
    current_action = ACTIONS.pop(0)

    while ((datetime.datetime.now() - start_time).total_seconds() < float(config.EXPERIMENT_MAX_DURATION_SECONDS)) and not rospy.is_shutdown():
        RUNNING_EXPERIMENT = True
        loop_start = rospy.Time.now()

        goal_not_reached = current_action.reached_goal_time is None

        TRAJECTORY_TRACKER.set_goal_position(current_action.goal_pose)
        goal_pose_publisher.publish(
            utils.pose_stamped_from_xyzrpy(
                xyzrpy=current_action.goal_pose,
                frame_id=config.PLATFORM_FRAME_ID,
                seq=0,
                stamp=rospy.Time.now()
            )
        )

        VELOCITY_HISTORY.append(LATEST_VELOCITY)
        
        latest_vel_avg = utils.average_velocity(VELOCITY_HISTORY, 2)

        if latest_imu_message is None:
            TRAJECTORY_TRACKER.set_current_velocity([latest_vel_avg[0], latest_vel_avg[1], 0.0, 0.0, 0.0, 0.0])
        else:
            TRAJECTORY_TRACKER.set_current_velocity([latest_vel_avg[0], latest_vel_avg[1], 0.0, 0.0, 0.0, -latest_imu_message.angular_velocity.z])

        robot_pose = utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE)
        robot_pose_xyzrpy = utils.to_xyzrpy(*robot_pose)

        BUILD_PLATFORM.publish_platform_transforms(robot_pose_xyzrpy, LATEST_MARKER_MESSAGE.header.stamp, transform_broadcaster)

        TRAJECTORY_TRACKER.set_latest_imu_reading(latest_imu_message)
        TRAJECTORY_TRACKER.set_current_position(robot_pose_xyzrpy)
        pose_error = TRAJECTORY_TRACKER.get_error()

        if not current_action.is_started:
            tolerance = config.TIGHT_POSE_TOLERANCE
            reached_goal = all([abs(error) < tol for (error, tol) in zip(pose_error, tolerance)])

            if current_action.action_type == 'open_gripper':
                if reached_goal:
                    GRIPPER_HANDLER.start_opening()
                    current_action.start()
                    TRAJECTORY_TRACKER.z_i = config.DEFAULT_Z_I_GAIN
                    TRAJECTORY_TRACKER.y_i = config.DEFAULT_Y_I_GAIN
                    TRAJECTORY_TRACKER.x_i = config.DEFAULT_X_I_GAIN
                    TRAJECTORY_TRACKER.clear_error_integrals()

            elif current_action.action_type == 'close_gripper':
                if reached_goal:
                    GRIPPER_HANDLER.start_closing()
                    current_action.start()
                    TRAJECTORY_TRACKER.z_i = config.BLOCK_HELD_Z_I_GAIN
                    TRAJECTORY_TRACKER.x_i = config.BLOCK_HELD_X_I_GAIN
                    TRAJECTORY_TRACKER.y_i = config.BLOCK_HELD_Y_I_GAIN
                    TRAJECTORY_TRACKER.clear_error_integrals()
            else:
                current_action.start()

            if current_action.is_started:
                rospy.loginfo("Starting action: {}".format(current_action))
        else:
            if current_action.is_complete(pose_error):
                rospy.loginfo("Completed action: {}. Error: {}".format(current_action, pose_error))

                if len(ACTIONS) > 0:
                    current_action = ACTIONS.pop(0)

        if current_action.is_started and current_action.is_complete(pose_error) and len(ACTIONS) == 0:
            rospy.loginfo("Completed all actions! Terminating")
            break

        if goal_not_reached and current_action.reached_goal_time is not None:
            rospy.loginfo("Reached goal position. Holding.")

        go_message = TRAJECTORY_TRACKER.get_next_rc_override()

        GRIPPER_HANDLER.update()
        GRIPPER_HANDLER.mix_into_rc_override_message(go_message)

        utils.terminate_if_unsafe(go_message, 150, DRY_RUN)
        rc_override_publisher.publish(go_message)

        loop_end = rospy.Time.now()
        loop_time = (loop_end - loop_start).to_sec()
        if (loop_time > 0.03):
            rospy.logwarn("Slow loop!")

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


def wait_for_marker_data():
    poll_rate = rospy.Rate(10)
    while TIMES_TRACKED_MARKER_SEEN < 100 and not rospy.is_shutdown():
        poll_rate.sleep()


def main():
    global DRY_RUN, goal_pose_publisher, transform_broadcaster
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

    rc_override_publisher = rospy.Publisher(
        "/mavros/rc/override",
        mavros_msgs.msg.OverrideRCIn,
        queue_size=1
    )

    goal_pose_publisher = rospy.Publisher(
        "/goal_pose",
        geometry_msgs.msg.PoseStamped,
        queue_size=1
    )

    transform_broadcaster = tf.TransformBroadcaster()

    try:
        DRY_RUN = rospy.get_param("~dry_run")
    except:
        rospy.logwarn("dry_run parameter not provided. Defaulting to test mode.")

    if DRY_RUN:
        rospy.loginfo("Running in test mode.")

    rospy.loginfo("Running control test experiment for {time} seconds".format(
        time=config.EXPERIMENT_MAX_DURATION_SECONDS
    ))

    rospy.loginfo("Waiting for enough marker data....")
    wait_for_marker_data()
    rospy.loginfo("Got marker data, going!!")

    run_build_plan(
        rc_override_publisher
    )

    rospy.loginfo("Finished experiment!")


if __name__ == "__main__":
    main()
