#! /usr/bin/python
import sys
import datetime
import os
import time
import random

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
import ballast_handler

import build_platform
import build_plan_parser

import stag_ros.srv

import droplet_underwater_assembly.msg


LATEST_MARKER_MESSAGE = None
RAW_VELOCITY_HISTORY = []
RUNNING_EXPERIMENT = False
VELOCITY_HISTORY = []
DRY_RUN = True
TIMES_TRACKED_MARKER_SEEN = 0
GRIPPER_HANDLER = gripper_handler.GripperHandler()
BALLAST_HANDLER = ballast_handler.BallastHandler()
#GRIPPER_HANDLER = None
IS_USING_OPEN_LOOP_CONTROL = False

BINARY_P_CONTROL_SELECTOR = 'BINARY_P'
OPEN_LOOP_CONTROL_SELECTOR = 'OPEN_LOOP'
PID_CONTROL_SELECTOR = 'PID'

ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR

SET_TRACKED_BUNDLE_IDS_PROXY = rospy.ServiceProxy("/stag_ros/set_tracked_bundle_ids", stag_ros.srv.SetTrackedBundles)

BUILD_PHASE_PUBLISHER = None

DISPLAY = None
try:
    DISPLAY = display_controller.DisplayController("/dev/ttyUSB0", 9600)
except:
    rospy.logerr("Unable to connect to display controller!")

LAST_TRACKED_MARKER_TIME = None

PLATFORMS = [
    build_platform.BuildPlatform(
        tag_id=0,
        slot_dimensions=(0.083, 0.083, 0.16),
        bottom_back_left_slot_location=(-1.96, -0.45, -0.44),
        dimensions=(7,7,3),
        center_back_pose=config.CENTER_BACK_POSE_RIGHT
    ),
    build_platform.BuildPlatform(
        tag_id=4,
        slot_dimensions=(0.083, 0.083, 0.16),
        #bottom_back_left_slot_location=(-1.88, -0.34, -0.42),
        bottom_back_left_slot_location=(-1.634, -0.132, -0.40),
        dimensions=(5,7,3),
        #center_back_pose=config.CENTER_BACK_POSE
        center_back_pose=[-2.10, -0.130, -0.15, 0.0, 0.0, 0.0]
    ),
]

latest_imu_message = None
goal_pose_publisher = None
transform_broadcaster = None

#pid_gains_dict = dict(
#    x_p=4.00,
#    y_p=4.00,
#    yaw_p=2.35, 
#    x_d=-1.0, 
#    y_d=-0.25,
#    yaw_d=1.0,
#    x_i=config.DEFAULT_X_I_GAIN,
#    y_i=config.DEFAULT_Y_I_GAIN,
#    yaw_i=0.15,
#    roll_p=-0.3,
#    roll_i=0.0,
#    roll_d=1.0,
#    z_p=3.5,
#    z_i=config.DEFAULT_Z_I_GAIN,
#    z_d=-0.50,
#    pitch_p=-1.0,
#    pitch_i=0.0,
#    pitch_d=0.75,
#) 
pid_gains_dict = dict(
    x_p=3.00,
    y_p=3.00,
    yaw_p=2.75, 
    x_d=0.0, 
    y_d=0.0,
    yaw_d=0.0,
    x_i=0.0,
    y_i=0.0,
    yaw_i=0.0,
    roll_p=2.0,
    roll_i=0.0,
    roll_d=0.0,
    z_p=3.5,
    z_i=0.0,
    z_d=-0.00,
    pitch_p=2.0,
    pitch_i=0.0,
    pitch_d=0.00,
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
    yaw_p=2.00, 
    x_d=0.0, 
    y_d=0.0,
    yaw_d=0.1,
    x_i=0,
    y_i=0,
    yaw_i=0.00,
    roll_p=-0.3,
    roll_i=0.0,
    roll_d=1.0,
    z_p=4.5,
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

#ACTIONS = [
#    assembly_action.AssemblyAction('change_platforms', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], to_platform_id=5),
#    assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.TIGHT_POSE_TOLERANCE),
#    assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.TIGHT_POSE_TOLERANCE),
#    #assembly_action.AssemblyAction('change_platforms', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], to_platform_id=5),
#    #assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.BINARY_P_POSE_TOLERANCE),
#    #assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE),
#    #assembly_action.AssemblyAction('change_platforms', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], to_platform_id=0),
#    #assembly_action.AssemblyAction('binary_P_move', config.CENTER_BACK_POSE, config.BINARY_P_POSE_TOLERANCE),
#    #assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE)
#]


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
                GRIPPER_HANDLER.start_opening_fingers()
                current_action.start()
                TRAJECTORY_TRACKER.z_i = config.DEFAULT_Z_I_GAIN
                TRAJECTORY_TRACKER.y_i = config.DEFAULT_Y_I_GAIN
                TRAJECTORY_TRACKER.x_i = config.DEFAULT_X_I_GAIN
                TRAJECTORY_TRACKER.clear_error_integrals()
                started = True

        elif current_action.action_type == 'close_gripper':
            ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR
            if reached_goal:
                GRIPPER_HANDLER.start_closing_fingers()
                current_action.start()
                TRAJECTORY_TRACKER.z_i = config.BLOCK_HELD_Z_I_GAIN
                TRAJECTORY_TRACKER.x_i = config.BLOCK_HELD_X_I_GAIN
                TRAJECTORY_TRACKER.y_i = config.BLOCK_HELD_Y_I_GAIN
                TRAJECTORY_TRACKER.clear_error_integrals()
                started = True

            if DISPLAY is not None:
                DISPLAY.update_led_state([255,125,0],1)

        elif current_action.action_type == 'change_platforms':
            config.TRACKED_MARKER_ID = current_action.to_platform_id
            ACTIVE_CONTROLLER = OPEN_LOOP_CONTROL_SELECTOR
            LAST_TRACKED_MARKER_TIME = None
            TRAJECTORY_TRACKER.clear_error_integrals()
            SET_TRACKED_BUNDLE_IDS_PROXY(
                tracked_bundle_ids=[current_action.to_platform_id]
            )

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
            rospy.loginfo("Completed action: {}. Error: {}. Tolerance {}".format(current_action, pose_error, current_action.pose_tolerance))

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


def publish_current_build_phase(current_action, started, current_location):
    info_message = droplet_underwater_assembly.msg.BuildPhase()
    info_message.header.stamp = rospy.Time.now()
    info_message.switched_this_frame = started
    info_message.current_action_is_started = current_action.is_started
    info_message.current_action = str(current_action)
    info_message.active_build_step = current_action.high_level_build_step
    info_message.goal_location = current_action.goal_pose
    info_message.current_location = current_location
    info_message.current_action_type = current_action.action_type
    info_message.move_tolerance = current_action.pose_tolerance
    info_message.action_sequence_id = current_action.sequence_id

    BUILD_PHASE_PUBLISHER.publish(info_message)


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
        GRIPPER_HANDLER.start_opening_fingers()

    start_time = datetime.datetime.now()
    number_actions = len(ACTIONS)
    current_action = ACTIONS.pop(0)

    while ((datetime.datetime.now() - start_time).total_seconds() < float(config.EXPERIMENT_MAX_DURATION_SECONDS)) and not rospy.is_shutdown():
        RUNNING_EXPERIMENT = True

        # checking if tag has been seen
        if (rospy.Time.now() - LATEST_MARKER_MESSAGE.header.stamp).to_sec() > config.MARKER_LOSS_TIME:
            rospy.logerr("No marker message gotten for {} seconds. Terminating!".format(config.MARKER_LOSS_TIME))
            break

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
                DISPLAY.update_lcd_display("{} / {}".format(current_action.sequence_id, number_actions), "{:0.1f} {:0.1f} {:0.2f}".format(*pose_error[3:6]))
                LAST_ERROR_DISPLAY = rospy.Time.now()

        changed = False
        changed, current_action = act_on_current_action(current_action, pose_error)

        if changed:
            update_active_controller(current_action)

        publish_current_build_phase(
            current_action,
            changed,
            get_robot_pose_xyzrpy()
        )

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
        BALLAST_HANDLER.update()

        utils.terminate_if_unsafe(go_message, 400, DRY_RUN)
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
        if DISPLAY is not None:
            if HAVE_ANY_MARKER_READING:
                DISPLAY.update_lcd_display("STag on.", "Waiting...")
            else:
                DISPLAY.update_lcd_display("No STag!", "No STag!")

        BALLAST_HANDLER.update()
        poll_rate.sleep()


def construct_tolerance_motion_experiment(cube_dimensions, cube_center, n_samples_per_tol, tol_step, tol_range, cutoff_seconds):
    actions = []

    initial_tolerance = [
        tol_range[1],
        tol_range[1],
        0.03,
        1000.0,
        1000.0,
        0.017
    ]

    tol_samples = []

    n_tolerance_steps = int((tol_range[1] - tol_range[0]) / tol_step)

    for tol_step_x in range(n_tolerance_steps):
        for tol_step_y in range(n_tolerance_steps):
            tol_samples.append(
                [
                    initial_tolerance[0] - float(tol_step_x) * tol_step,
                    initial_tolerance[1] - float(tol_step_y) * tol_step,
                    initial_tolerance[2],
                    initial_tolerance[3],
                    initial_tolerance[4],
                    initial_tolerance[5],
                ]
            )

    rospy.loginfo("Sampling {} tolerances".format(tol_samples))

    goal_moves_with_tol = []
    
    pt_1 = list(cube_center)
    pt_1[1] = pt_1[1] + 0.20
    pt_2 = list(cube_center)
    pt_2[0] = pt_2[0] + 0.20

    move_triangle = [
        cube_center,
        pt_1,
        pt_2
    ]

    assert(n_samples_per_tol == 3)
    for sample in tol_samples:
        for goal_idx in range(n_samples_per_tol):
            next_point = move_triangle[goal_idx]

            goal_move = [
                next_point[0],
                next_point[1],
                next_point[2],
                0.0,
                0.0,
                0.0,
            ]

            goal_moves_with_tol.append(
                (goal_move, sample)
            )

    rospy.loginfo("Number moves {}".format(len(goal_moves_with_tol)))
    new_action = assembly_action.AssemblyAction(
        action_type="move",
        goal_pose=[move_triangle[2][0], move_triangle[2][1], move_triangle[2][2], 0.0, 0.0, 0.0],
        pose_tolerance=initial_tolerance,
        position_hold_time=6.0
    )

    for move, tol in goal_moves_with_tol:
        new_action = assembly_action.AssemblyAction(
            action_type="move",
            goal_pose=move,
            pose_tolerance=tol,
            position_hold_time=6.0
        )

        actions.append(new_action)

        new_action = assembly_action.AssemblyAction(
            action_type="move",
            goal_pose=move,
            pose_tolerance=tol,
            position_hold_time=0.0
        )

        new_action.timeout = cutoff_seconds
        actions.append(new_action)

    for idx, action in enumerate(actions):
        action.high_level_build_step = "TOL_SAMPLING"
        action.sequence_id = idx

    return actions


def main():
    global PLATFORMS, ACTIONS, DRY_RUN, goal_pose_publisher, transform_broadcaster, BUILD_PHASE_PUBLISHER
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

    BUILD_PHASE_PUBLISHER = rospy.Publisher(
        config.BUILD_PHASE_TOPIC,
        droplet_underwater_assembly.msg.BuildPhase,
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

    #ACTIONS = construct_tolerance_motion_experiment(
    #    cube_dimensions=[0.5,0.5,0.5,0.15],
    #    cube_center=[-1.6, -0.3, -0.3, 0.0],
    #    n_samples_per_tol=3,
    #    tol_step=0.0025,
    #    tol_range=[0.0075,0.03],
    #    cutoff_seconds=90.0
    #)

    rospy.loginfo("Actions to complete:")
    for idx, action in enumerate(ACTIONS):
        rospy.loginfo("    {}: {}".format(idx, action))

    rospy.loginfo("Pauing for display initialization...")
    time.sleep(10)
    if DISPLAY is not None:
        DISPLAY.update_led_state([0, 255, 0], 1)       

    rospy.loginfo("Waiting for enough marker data from marker {}....".format(config.TRACKED_MARKER_ID))
    BALLAST_HANDLER.start_emptying_ballast_air()

    wait_for_marker_data()
    rospy.loginfo("Got marker data, going!!")

    for action in ACTIONS:
        action.gripper_handler = GRIPPER_HANDLER
        
        if action.high_level_build_step is None:
            raise Exception("Action {} is not associated with a high level build step!".format(action))

    SET_TRACKED_BUNDLE_IDS_PROXY(
        tracked_bundle_ids=[config.TRACKED_MARKER_ID]
    )

    BALLAST_HANDLER.start_emptying_ballast_air()
    run_build_plan(
        rc_override_publisher
    )

    rospy.loginfo("Finished experiment!")
    BALLAST_HANDLER.start_emptying_ballast_air()


if __name__ == "__main__":
    main()
