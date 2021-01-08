#! /usr/bin/python
import sys
import datetime
import os
import time

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
import display_controller

import build_platform
import build_plan_parser


LATEST_MARKER_MESSAGE = None
RAW_VELOCITY_HISTORY = []
RUNNING_EXPERIMENT = False
VELOCITY_HISTORY = []
DRY_RUN = True
TIMES_TRACKED_MARKER_SEEN = 0
GRIPPER_HANDLER = gripper_handler.GripperHandler()
IS_USING_OPEN_LOOP_CONTROL = False

BINARY_P_CONTROL_SELECTOR = 'BINARY_P'
OPEN_LOOP_CONTROL_SELECTOR = 'OPEN_LOOP'
PID_CONTROL_SELECTOR = 'PID'

ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR

DISPLAY = None
try:
    DISPLAY = display_controller.DisplayController("/dev/ttyUSB0", 9600)
except:
    rospy.logerr("Unable to connect to display controller!")

LAST_TRACKED_MARKER_TIME = None

PLATFORMS = [
    build_platform.BuildPlatform(
        tag_id=0,
        slot_dimensions=(0.083, 0.083, 0.19),
        bottom_back_left_slot_location=(-1.58, -0.05, -0.15),
        dimensions=(3,3,3),
        center_back_pose=config.CENTER_BACK_POSE
    ),
    build_platform.BuildPlatform(
        tag_id=3,
        slot_dimensions=(0.083, 0.083, 0.19),
        bottom_back_left_slot_location=(-1.58, 0.42, -0.15),
        dimensions=(3,3,3),
        center_back_pose=config.CENTER_BACK_POSE
    ),
]

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
    **pid_gains_dict
)

ERROR_DISPLAY_RATE = 0.75
LAST_ERROR_DISPLAY = None

OPEN_LOOP_TRACKER = trajectory_tracker.OpenLoopTracker(
    **pid_gains_dict
)

binary_gains = dict(
    x_p=3.00,
    y_p=3.00,
    yaw_p=3.00, 
    x_d=0.0, 
    y_d=0.0,
    yaw_d=1.0,
    x_i=0,
    y_i=0,
    yaw_i=0.00,
    roll_p=-0.3,
    roll_i=0.0,
    roll_d=1.0,
    z_p=20.5,
    z_i=0.0,
    z_d=0.00,
    pitch_p=-2.0,
    pitch_i=0.0,
    pitch_d=0.75,
) 

BINARY_P_TRACKER = trajectory_tracker.PIDTracker(
    **binary_gains
)

LATEST_VELOCITY = None

TRAJECTORY_TRACKER = CLOSED_LOOP_TRACKER

HAVE_ANY_MARKER_READING = False

# gonna need many build platforms. Maybe I can wait on that?

ACTIONS = [
    assembly_action.AssemblyAction('change_platforms', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], to_platform_id=5),
    assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
    assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.TIGHT_POSE_TOLERANCE),
    assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.TIGHT_POSE_TOLERANCE),
    #assembly_action.AssemblyAction('change_platforms', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], to_platform_id=5),
    #assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.BINARY_P_POSE_TOLERANCE),
    #assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE),
    #assembly_action.AssemblyAction('change_platforms', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], to_platform_id=0),
    #assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.BINARY_P_POSE_TOLERANCE),
    #assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE)
]


def imu_callback(imu_message):
    global latest_imu_message
    latest_imu_message = imu_message

    TRAJECTORY_TRACKER.set_latest_imu_reading(latest_imu_message)


def marker_callback(marker_message):
    global LATEST_MARKER_MESSAGE, TIMES_TRACKED_MARKER_SEEN, LATEST_VELOCITY, RAW_VELOCITY_HISTORY, LAST_TRACKED_MARKER_TIME, HAVE_ANY_MARKER_READING

    HAVE_ANY_MARKER_READING = True

    if LATEST_MARKER_MESSAGE is not None:
        last_marker_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE))

    for marker in marker_message.markers:
        if marker.id == config.TRACKED_MARKER_ID:
            #LAST_TRACKED_MARKER_TIME = marker_message.header.stamp
            LAST_TRACKED_MARKER_TIME = rospy.Time.now()

            if LATEST_MARKER_MESSAGE is not None:
                current_marker_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(marker))
                distance_travelled = utils.get_error(last_marker_pose, current_marker_pose)
                time_delta_seconds = (marker.header.stamp - LATEST_MARKER_MESSAGE.header.stamp).to_sec()
                LATEST_VELOCITY = [value / time_delta_seconds for value in distance_travelled]
                RAW_VELOCITY_HISTORY.append(LATEST_VELOCITY)

            LATEST_MARKER_MESSAGE = marker
            TIMES_TRACKED_MARKER_SEEN = TIMES_TRACKED_MARKER_SEEN + 1


def get_latest_velocity_xyzrpy():
    latest_vel_avg = utils.average_velocity(RAW_VELOCITY_HISTORY, 2)

    if latest_imu_message is None:
        return [latest_vel_avg[0], latest_vel_avg[1], 0.0, 0.0, 0.0, 0.0]

    return [latest_vel_avg[0], latest_vel_avg[1], 0.0, 0.0, 0.0, -latest_imu_message.angular_velocity.z]


def get_robot_pose_xyzrpy():
    if LATEST_MARKER_MESSAGE is None:
        return [0.0] * 6

    robot_pose = utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE)
    robot_pose_xyzrpy = utils.to_xyzrpy(*robot_pose)
    
    return robot_pose_xyzrpy


def update_closed_loop_controller(current_action):
    TRAJECTORY_TRACKER.set_goal_position(current_action.goal_pose)
    goal_pose_publisher.publish(
        utils.pose_stamped_from_xyzrpy(
            xyzrpy=current_action.goal_pose,
            frame_id="/world",
            seq=0,
            stamp=rospy.Time.now()
        )
    )


    latest_vel_avg = get_latest_velocity_xyzrpy()

    if LATEST_VELOCITY is not None:
        VELOCITY_HISTORY.append(LATEST_VELOCITY)
    
    latest_vel_avg = get_latest_velocity_xyzrpy()
    robot_pose_xyzrpy = get_robot_pose_xyzrpy()

    TRAJECTORY_TRACKER.set_latest_imu_reading(latest_imu_message)
    TRAJECTORY_TRACKER.set_current_position(robot_pose_xyzrpy)
    TRAJECTORY_TRACKER.set_current_velocity(latest_vel_avg)


def update_open_loop_controller(current_action):
    TRAJECTORY_TRACKER.set_goal_position(current_action.goal_pose)
    latest_vel_avg = get_latest_velocity_xyzrpy()
    robot_pose_xyzrpy = get_robot_pose_xyzrpy()

    OPEN_LOOP_TRACKER.set_current_position(robot_pose_xyzrpy)
    OPEN_LOOP_TRACKER.set_latest_imu_reading(latest_imu_message)
    OPEN_LOOP_TRACKER.set_current_velocity(latest_vel_avg)


def update_binary_P_controller(current_action):
    BINARY_P_TRACKER.set_goal_position(current_action.goal_pose)
    goal_pose_publisher.publish(
        utils.pose_stamped_from_xyzrpy(
            xyzrpy=current_action.goal_pose,
            frame_id="/world",
            seq=0,
            stamp=rospy.Time.now()
        )
    )


    latest_vel_avg = get_latest_velocity_xyzrpy()

    if LATEST_VELOCITY is not None:
        VELOCITY_HISTORY.append(LATEST_VELOCITY)
    
    latest_vel_avg = get_latest_velocity_xyzrpy()
    robot_pose_xyzrpy = get_robot_pose_xyzrpy()

    BINARY_P_TRACKER.set_latest_imu_reading(latest_imu_message)
    BINARY_P_TRACKER.set_current_position(robot_pose_xyzrpy)
    BINARY_P_TRACKER.set_current_velocity(latest_vel_avg)

# how do we want to decide when the open loop controller should stop?

def act_on_current_action(current_action, pose_error):
    global ACTIVE_CONTROLLER, LAST_TRACKED_MARKER_TIME

    goal_not_reached = current_action.reached_goal_time is None

    started = False
    if not current_action.is_started:
        tolerance = config.TIGHT_POSE_TOLERANCE
        reached_goal = all([abs(error) < tol for (error, tol) in zip(pose_error, tolerance)])

        if current_action.action_type == 'open_gripper':
            ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR
            if reached_goal:
                GRIPPER_HANDLER.start_opening()
                current_action.start()
                TRAJECTORY_TRACKER.z_i = config.DEFAULT_Z_I_GAIN
                TRAJECTORY_TRACKER.y_i = config.DEFAULT_Y_I_GAIN
                TRAJECTORY_TRACKER.x_i = config.DEFAULT_X_I_GAIN
                TRAJECTORY_TRACKER.clear_error_integrals()
                started = True

        elif current_action.action_type == 'close_gripper':
            ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR
            if reached_goal:
                GRIPPER_HANDLER.start_closing()
                current_action.start()
                TRAJECTORY_TRACKER.z_i = config.BLOCK_HELD_Z_I_GAIN
                TRAJECTORY_TRACKER.x_i = config.BLOCK_HELD_X_I_GAIN
                TRAJECTORY_TRACKER.y_i = config.BLOCK_HELD_Y_I_GAIN
                TRAJECTORY_TRACKER.clear_error_integrals()
                started = True

        elif current_action.action_type == 'change_platforms':
            config.TRACKED_MARKER_ID = current_action.to_platform_id
            ACTIVE_CONTROLLER = OPEN_LOOP_CONTROL_SELECTOR
            LAST_TRACKED_MARKER_TIME = None
            TRAJECTORY_TRACKER.clear_error_integrals()

            if DISPLAY is not None:
                DISPLAY.update_led_state([255,255,0],1)
            current_action.start()
            started = True
        elif current_action.action_type == 'move':
            ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR

            if DISPLAY is not None:
                DISPLAY.update_led_state([255,0,255],1)
            current_action.start()
            started = True
        elif current_action.action_type == 'binary_P_move':
            ACTIVE_CONTROLLER = BINARY_P_CONTROL_SELECTOR
            if DISPLAY is not None:
                DISPLAY.update_led_state([255,0,0],1)
            current_action.start()
            started = True
        elif current_action.action_type == 'move_wrist':
            ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR
            if DISPLAY is not None:
                DISPLAY.update_led_state([0,0,255],1)
            
            GRIPPER_HANDLER.desired_rotation_position = current_action.wrist_rotation_pwm

            current_action.start()
            started = True
        else:
            raise Exception("Unrecognized action type!")

        if current_action.is_started:
            rospy.loginfo("Starting action: {}".format(current_action))
    else:
        if current_action.is_complete(pose_error, last_tracked_marker_time=LAST_TRACKED_MARKER_TIME):
            rospy.loginfo("Completed action: {}. Error: {}".format(current_action, pose_error))

            if len(ACTIONS) > 0:
                return True, ACTIONS.pop(0)

    if goal_not_reached and current_action.reached_goal_time is not None:
        rospy.loginfo("Reached goal position. Holding.")

    return started, current_action


def update_active_controller(current_action):
    if ACTIVE_CONTROLLER == OPEN_LOOP_CONTROL_SELECTOR: 
        update_open_loop_controller(current_action)
    elif ACTIVE_CONTROLLER == PID_CONTROL_SELECTOR:
        update_closed_loop_controller(current_action)
    elif ACTIVE_CONTROLLER == BINARY_P_CONTROL_SELECTOR:
        update_binary_P_controller(current_action)
    else:
        raise Exception("Unrecognized active control selector!")


def run_build_plan(rc_override_publisher):
    global latest_imu_message, VELOCITY_HISTORY, RUNNING_EXPERIMENT, LAST_ERROR_DISPLAY

    if DISPLAY is not None:
        DISPLAY.update_buzzer_pattern(1)

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

        update_active_controller(current_action)

        goal_not_reached = current_action.reached_goal_time is None
        pose_error = None

        if ACTIVE_CONTROLLER == OPEN_LOOP_CONTROL_SELECTOR: 
            pose_error = OPEN_LOOP_TRACKER.get_error()
        elif ACTIVE_CONTROLLER == PID_CONTROL_SELECTOR:
            pose_error = CLOSED_LOOP_TRACKER.get_error()
        elif ACTIVE_CONTROLLER == BINARY_P_CONTROL_SELECTOR:
            pose_error = BINARY_P_TRACKER.get_error()
        else:
            raise Exception("Unrecognized active control selector!")
        if DISPLAY is not None:
            if LAST_ERROR_DISPLAY is None or (rospy.Time.now() - LAST_ERROR_DISPLAY).to_sec() > ERROR_DISPLAY_RATE:
                DISPLAY.update_lcd_display("{:0.2f} {:0.2f} {:0.2f}".format(*pose_error[0:3]), "{:0.2f} {:0.2f} {:0.2f}".format(*pose_error[3:6]))
                LAST_ERROR_DISPLAY = rospy.Time.now()

        changed = False
        changed, current_action = act_on_current_action(current_action, pose_error)

        if changed:
            update_active_controller(current_action)

        if current_action.is_started and current_action.is_complete(pose_error, last_tracked_marker_time=LAST_TRACKED_MARKER_TIME) and len(ACTIONS) == 0:
            rospy.loginfo("Completed all actions! Terminating")
            break

        if ACTIVE_CONTROLLER == OPEN_LOOP_CONTROL_SELECTOR: 
            go_message = OPEN_LOOP_TRACKER.get_next_rc_override()
        elif ACTIVE_CONTROLLER == PID_CONTROL_SELECTOR:
            go_message = CLOSED_LOOP_TRACKER.get_next_rc_override()
        elif ACTIVE_CONTROLLER == BINARY_P_CONTROL_SELECTOR:
            go_message = BINARY_P_TRACKER.get_next_rc_override()
        else:
            raise Exception("Unrecognized active control selector!")

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
    poll_rate = rospy.Rate(2)
    while TIMES_TRACKED_MARKER_SEEN < 100 and not rospy.is_shutdown():
        if DISPLAY is not None:
            if HAVE_ANY_MARKER_READING:
                DISPLAY.update_lcd_display("STag on.", "Waiting...")
            else:
                DISPLAY.update_lcd_display("No STag!", "No STag!")

        poll_rate.sleep()


def main():
    global PLATFORMS, ACTIONS, DRY_RUN, goal_pose_publisher, transform_broadcaster
    rospy.init_node("droplet_underwater_assembly")

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

    try:
        build_plan_file = rospy.get_param("~build_plan_file")
    except:
        rospy.logerr("The build_plan_file parameter is required to launch the code.")

    if DRY_RUN:
        rospy.loginfo("Running in test mode.")

    rospy.loginfo("Running control test experiment for {time} seconds".format(
        time=config.EXPERIMENT_MAX_DURATION_SECONDS
    ))

    parser = build_plan_parser.BuildPlanParser(
        build_platforms=PLATFORMS,
        high_offset=config.HIGH_Z_OFFSET,
        mid_low_offset=config.MID_LOW_Z_OFFSET
    )

    ACTIONS = parser.construct_actions_from_text(
        build_plan_file=build_plan_file,
        start_platform=config.TRACKED_MARKER_ID,
    )

    rospy.loginfo("Actions to complete:")
    for idx, action in enumerate(ACTIONS):
        rospy.loginfo("    {}: {}".format(idx, action))

    rospy.loginfo("Pauing for display initialization...")
    time.sleep(10)
    if DISPLAY is not None:
        DISPLAY.update_led_state([0, 255, 0], 1)       

    rospy.loginfo("Waiting for enough marker data....")
    wait_for_marker_data()
    rospy.loginfo("Got marker data, going!!")

    for action in ACTIONS:
        action.gripper_handler = GRIPPER_HANDLER

    run_build_plan(
        rc_override_publisher
    )

    rospy.loginfo("Finished experiment!")


if __name__ == "__main__":
    main()
