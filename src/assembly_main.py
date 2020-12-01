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


LATEST_MARKER_MESSAGE = None
RAW_VELOCITY_HISTORY = []
RUNNING_EXPERIMENT = False
VELOCITY_HISTORY = []
DRY_RUN = True
TIMES_TRACKED_MARKER_SEEN = 0
GRIPPER_HANDLER = gripper_handler.GripperHandler()
IS_USING_OPEN_LOOP_CONTROL = False

LAST_TRACKED_MARKER_TIME = None

latest_imu_message = None
goal_pose_publisher = None
transform_broadcaster = None

pid_gains_dict = dict(
    x_p=4.00,
    y_p=4.00,
    yaw_p=2.35, 
    x_d=-1.0, 
    y_d=-0.25,
    yaw_d=1.0,
    x_i=config.DEFAULT_X_I_GAIN,
    y_i=config.DEFAULT_Y_I_GAIN,
    yaw_i=0.08,
    roll_p=-0.3,
    roll_i=0.0,
    roll_d=1.0,
    z_p=3.5,
    z_i=config.DEFAULT_Z_I_GAIN,
    z_d=-0.50,
    pitch_p=-1.0,
    pitch_i=0.0,
    pitch_d=0.75,
) 

CLOSED_LOOP_TRACKER = trajectory_tracker.PIDTracker(
    **gains_dict
)

OPEN_LOOP_TRACKER = trajectory_tracker.OpenLoopTracker(
    **gains_dict
)

TRAJECTORY_TRACKER = CLOSED_LOOP_TRACKER

BUILD_PLATFORM = build_platform.BuildPlatform(
    pickup_dimensions=config.PICKUP_PLATFORM_DIMENSIONS,
    drop_dimensions=config.DROP_PLATFORM_DIMENSIONS,
    row_spacing=config.SLOT_Z_STRIDE,
    column_spacing=config.SLOT_X_STRIDE,
    min_pickup_slot=config.MIN_PICKUP_SLOT,
    min_drop_slot=config.MIN_DROP_SLOT,
    frame_id="/build_platform"
)

ACTIONS = BUILD_PLATFORM.convert_build_steps_into_assembly_actions(config.BUILD_PLAN, GRIPPER_HANDLER)


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
            LAST_TRACKED_MARKER_TIME = marker_message.header.stamp

            if LATEST_MARKER_MESSAGE is not None:
                current_marker_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(marker))
                distance_travelled = utils.get_error(last_marker_pose, current_marker_pose)
                time_delta_seconds = (marker.header.stamp - LATEST_MARKER_MESSAGE.header.stamp).to_sec()
                LATEST_VELOCITY = [value / time_delta_seconds for value in distance_travelled]

            LATEST_MARKER_MESSAGE = marker
            TIMES_TRACKED_MARKER_SEEN = TIMES_TRACKED_MARKER_SEEN + 1

            if RUNNING_EXPERIMENT:
                RAW_VELOCITY_HISTORY.append(LATEST_VELOCITY)

            if latest_imu_message is not None:
                robot_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE))
                BUILD_PLATFORM.update_platform_roll_pitch_estimate(robot_pose, latest_imu_message)


def get_latest_velocity_xyzrpy():
    latest_vel_avg = utils.average_velocity(VELOCITY_HISTORY, 2)

    if latest_imu_message is None:
        return [latest_vel_avg[0], latest_vel_avg[1], 0.0, 0.0, 0.0, 0.0]

    return [latest_vel_avg[0], latest_vel_avg[1], 0.0, 0.0, 0.0, -latest_imu_message.angular_velocity.z]

def get_robot_pose_xyzrpy():
    robot_pose = utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE)
    robot_pose_xyzrpy = utils.to_xyzrpy(*robot_pose)
    
    return robot_pose_xyzrpy


def update_closed_loop_controller():
        TRAJECTORY_TRACKER.set_goal_position(current_action.goal_pose)
        goal_pose_publisher.publish(
            utils.pose_stamped_from_xyzrpy(
                xyzrpy=current_action.goal_pose,
                frame_id=BUILD_PLATFORM.frame_id,
                seq=0,
                stamp=rospy.Time.now()
            )
        )

        latest_vel_avg = get_latest_velocity_xyzrpy()

        VELOCITY_HISTORY.append(LATEST_VELOCITY)
        
        latest_vel_avg = get_latest_velocity_xyzrpy()
        robot_pose_xyzrpy = get_robot_pose_xyzrpy()

        TRAJECTORY_TRACKER.set_latest_imu_reading(latest_imu_message)
        TRAJECTORY_TRACKER.set_current_position(robot_pose_xyzrpy)
        TRAJECTORY_TRACKER.set_current_velocity(latest_vel_avg)


def update_open_loop_controller():
    latest_vel_avg = get_latest_velocity_xyzrpy()
    robot_pose_xyzrpy = get_robot_pose_xyzrpy()

    OPEN_LOOP_TRACKER.set_latest_imu_reading(latest_imu_message)
    OPEN_LOOP_TRACKER.set_current_position(robot_pose_xyzrpy)
    OPEN_LOOP_TRACKER.set_current_velocity(latest_vel_avg)

# how do we want to decide when the open loop controller should stop?

def act_on_current_action(current_action):
    global IS_USING_OPEN_LOOP_CONTROL

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

        elif current_action.action_type == 'change_platforms':
            TRACKED_MARKER_ID = current_action.to_platform_id
            IS_USING_OPEN_LOOP_CONTROL = True
            current_action.start()
        else:
            current_action.start()

        if current_action.is_started:
            rospy.loginfo("Starting action: {}".format(current_action))

        if current_action.action_type != 'change_platforms':
            IS_USING_OPEN_LOOP_CONTROL = False
    else:
        if current_action.is_complete(pose_error, last_tracked_marker_time=LAST_TRACKED_MARKER_TIME):
            rospy.loginfo("Completed action: {}. Error: {}".format(current_action, pose_error))

            if len(ACTIONS) > 0:
                return ACTIONS.pop(0)

    if goal_not_reached and current_action.reached_goal_time is not None:
        rospy.loginfo("Reached goal position. Holding.")

    return current_action


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

        update_controller()

        goal_not_reached = current_action.reached_goal_time is None
        pose_error = TRAJECTORY_TRACKER.get_error()

        current_action = act_on_current_action(current_action, pose_error)

        if current_action.is_started and current_action.is_complete(pose_error, LAST_TRACKED_MARKER_TIME) and len(ACTIONS) == 0:
            rospy.loginfo("Completed all actions! Terminating")
            break

        if IS_USING_OPEN_LOOP_CONTROL:
            go_message = OPEN_LOOP_TRACKER.get_next_rc_override()
        else:
            go_message = TRAJECTORY_TRACKER.get_next_rc_override()

        GRIPPER_HANDLER.update()
        GRIPPER_HANDLER.mix_into_rc_override_message(go_message)

        utils.terminate_if_unsafe(go_message, 150, DRY_RUN)
        rc_override_publisher.publish(go_message)

        loop_end = rospy.Time.now()
        loop_time = (loop_end - loop_start).to_sec()
        if (loop_time > 0.03):
            rospy.logwarn("Slow loop! {}".format(loop_time))

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
        config.AR_MARKER_TOPIC,
        stag_ros.msg.StagMarkers,
        marker_callback,
        queue_size=1
    )

    imu_subscriber = rospy.Subscriber(
        config.IMU_TOPIC,
        sensor_msgs.msg.Imu,
        imu_callback,
        queue_size=1
    )

    rc_override_publisher = rospy.Publisher(
        config.RC_OVERRIDE_TOPIC,
        mavros_msgs.msg.OverrideRCIn,
        queue_size=1
    )

    goal_pose_publisher = rospy.Publisher(
        config.GOAL_POSE_TOPIC,
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

    rospy.loginfo("Running build plan:")
    for idx, step in enumerate(config.BUILD_PLAN):
        rospy.loginfo("    {}: {}".format(idx, step))

    rospy.loginfo("Actions to complete:")
    for idx, action in enumerate(ACTIONS):
        rospy.loginfo("    {}: {}".format(idx, action))

    rospy.loginfo("Waiting for enough marker data....")
    wait_for_marker_data()
    rospy.loginfo("Got marker data, going!!")

    run_build_plan(
        rc_override_publisher
    )

    rospy.loginfo("Finished experiment!")


if __name__ == "__main__":
    main()
