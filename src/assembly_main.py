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
import tf.transformations

import geometry_msgs.msg
import mavros_msgs.msg
import stag_ros.msg
import sensor_msgs.msg
import std_msgs.msg

from droplet_underwater_assembly_libs import utils
from droplet_underwater_assembly_libs import gripper_handler
from droplet_underwater_assembly_libs import trajectory_tracker
from droplet_underwater_assembly_libs import assembly_action
from droplet_underwater_assembly_libs import config
from droplet_underwater_assembly_libs import display_controller
from droplet_underwater_assembly_libs import ballast_handler
from droplet_underwater_assembly_libs import build_platform
from droplet_underwater_assembly_libs import build_plan_parser

import stag_ros.srv

import droplet_underwater_assembly.msg

# should bailing release be a new type of action or just a couple of options on the release action
# bailing release is
#   1. turn thrusters on pushing down. 
#   2. bailing out air for a fixed amount of time.
#   3. not pushing any direction other than down. What are the options that are needed? how hard to push down (done by changing the previous setpoint). Can implement the not pushing thing by just zeroing out gains.
# I think there are a lot of options that would get clunky elsehwere because we want the standard options when dropping a code

INPUT_LED_ON = False

LATEST_MARKER_MESSAGE = None
RAW_VELOCITY_HISTORY = []
RUNNING_EXPERIMENT = False
VELOCITY_HISTORY = []
DRY_RUN = True
TIMES_TRACKED_MARKER_SEEN = 0
GRIPPER_HANDLER = gripper_handler.GripperHandler()
BALLAST_HANDLER = ballast_handler.BallastHandler()

BINARY_P_CONTROL_SELECTOR = 'BINARY_P'
OPEN_LOOP_CONTROL_SELECTOR = 'OPEN_LOOP'
PID_CONTROL_SELECTOR = 'PID'
GRASP_SELECTOR = 'GRASP'
BUOYANCY_CHANGE_CONTROL_SELECTOR = 'BUOYANCY_CHANGE'
LEFT_RIGHT_MOVE_CONTROL_SELECTOR = 'LEFT_RIGHT_MOVE'

ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR

SET_TRACKED_BUNDLE_IDS_PROXY = rospy.ServiceProxy(
    "/stag_ros/set_tracked_bundle_ids",
    stag_ros.srv.SetTrackedBundles
)

BUILD_PHASE_PUBLISHER = None

DISPLAY = None
try:
    DISPLAY = display_controller.DisplayController("/dev/arduino-serial-ui", 9600)
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
latest_pressure_message = None
goal_pose_publisher = None
transform_broadcaster = None

DEFAULT_Z_P = 1.0 
pid_gains_dict = dict(
    x_p=2.50,
    y_p=2.50,
    yaw_p=2.00, 
    x_d=-0.7, 
    y_d=-0.25,
    yaw_d=1.0,
    x_i=config.DEFAULT_X_I_GAIN,
    y_i=config.DEFAULT_Y_I_GAIN,
    yaw_i=0.15,
    roll_p=1.0,
    roll_i=0.1,
    roll_d=-0.50,
    z_p=DEFAULT_Z_P,
    z_i=config.DEFAULT_Z_I_GAIN,
    z_d=0.00,
    pitch_p=-1.0,
    pitch_i=-0.1,
    pitch_d=0.50,
)

left_right_move_gains_dict = dict(
    x_p=2.50,
    y_p=2.50,
    yaw_p=2.00, 
    x_d=-0.7, 
    y_d=-0.25,
    yaw_d=1.0,
    x_i=0.0,
    y_i=0.0,
    yaw_i=0.25,
    roll_p=1.0,
    roll_i=0.0,
    roll_d=-0.50,
    z_p=DEFAULT_Z_P,
    z_i=0.0,
    z_d=0.00,
    pitch_p=-1.0,
    pitch_i=-0.0,
    pitch_d=0.50,
)
#pid_gains_dict = dict( # single gain for debugging
#    x_p=0.00,
#    y_p=0.00,
#    yaw_p=0.00, 
#    x_d=0.0, 
#    y_d=0.0,
#    yaw_d=0.0,
#    x_i=0.0,
#    y_i=0.0,
#    yaw_i=0.0,
#    roll_p=1.25,
#    roll_i=0.0,
#    roll_d=0.0,
#    z_p=0.0,
#    z_i=0.0,
#    z_d=0.00,
#    pitch_p=0.0,
#    pitch_i=0.0,
#    pitch_d=0.00,
#) 

CLOSED_LOOP_TRACKER = trajectory_tracker.PIDTracker(
    **pid_gains_dict
)

LEFT_RIGHT_MOVE_TRACKER = trajectory_tracker.PIDTracker(
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
    roll_d=0.0,
    z_p=4.5,
    z_i=0.0,
    z_d=0.00,
    pitch_p=-2.0,
    pitch_i=0.0,
    pitch_d=0.00,
) 

#buoyancy_change_gains = dict(
#    x_p=3.00,
#    y_p=3.00,
#    yaw_p=2.35, 
#    x_d=-1.0, 
#    y_d=-0.25
#    yaw_d=1.0,
#    x_i=config.DEFAULT_X_I_GAIN,
#    y_i=config.DEFAULT_Y_I_GAIN,
#    yaw_i=0.15,
#    roll_p=1.0,
#    roll_i=0.0,
#    roll_d=-0.50,
#    z_p=0.70,
#    z_i=0.0,
#    z_d=0.00,
#    pitch_p=-1.0,
#    pitch_i=0.0,
#    pitch_d=0.50,
#)
buoyancy_change_gains = dict(
    x_p=2.00,
    y_p=2.00,
    yaw_p=2.40, 
    x_d=-0.0, 
    y_d=-0.00,
    yaw_d=1.0,
    x_i=0.0,
    y_i=0.0,
    yaw_i=0.00,
    roll_p=2.0,
    roll_i=0.05,
    roll_d=-0.00,
    z_p=0.77,
    z_i=0.0,
    z_d=0.00,
    pitch_p=-2.0,
    pitch_i=-0.05,
    pitch_d=0.50,
)

BINARY_P_TRACKER = trajectory_tracker.PIDTracker(
    **binary_gains
)

BUOYANCY_CHANGE_TRACKER = trajectory_tracker.PIDTracker(
    **buoyancy_change_gains
)

LATEST_VELOCITY = None
HAVE_ANY_MARKER_READING = False

# hack used for led indicator thing for charlie's sunflower stuff
SHOULD_CANCEL_HOLD = False
number_on_frames = 0
def light_flash_callback(flash_message):
    global SHOULD_CANCEL_HOLD, number_on_frames

    if flash_message.data:
        number_on_frames = number_on_frames + 1
    else:
        if number_on_frames > 0:
            rospy.loginfo("Flash has turned off!")
        SHOULD_CANCEL_HOLD = False
        number_on_frames = 0

    if number_on_frames == 30:
        rospy.loginfo("Found a flash on for long enough. Cancelling current action!!")
        SHOULD_CANCEL_HOLD = True


def pressure_callback(pressure_message):
    global latest_pressure_message
    latest_pressure_message = pressure_message


def imu_callback(imu_message):
    global latest_imu_message
    latest_imu_message = imu_message

    CLOSED_LOOP_TRACKER.set_latest_imu_reading(latest_imu_message)


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
    CLOSED_LOOP_TRACKER.set_goal_position(current_action.goal_pose)
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

    CLOSED_LOOP_TRACKER.set_latest_imu_reading(latest_imu_message)
    CLOSED_LOOP_TRACKER.set_current_position(robot_pose_xyzrpy)
    CLOSED_LOOP_TRACKER.set_current_velocity(latest_vel_avg)


def update_left_right_move_controller(current_action):
    global latest_imu_message, latest_pressure_message

    LEFT_RIGHT_MOVE_TRACKER.set_goal_position(current_action.goal_pose)

    imu_orientation = [
        latest_imu_message.orientation.x,
        latest_imu_message.orientation.y,
        latest_imu_message.orientation.z,
        latest_imu_message.orientation.w
    ]

    yaw, roll, pitch = tf.transformations.euler_from_quaternion(
        imu_orientation,
        axes='szyx'
    )

    robot_pose_xyzrpy = get_robot_pose_xyzrpy()

    position = [
        robot_pose_xyzrpy[0],
        robot_pose_xyzrpy[1],
        robot_pose_xyzrpy[2],
        roll,
        pitch,
        robot_pose_xyzrpy[5]
    ]

    latest_vel_avg = get_latest_velocity_xyzrpy()

    if LATEST_VELOCITY is not None:
        VELOCITY_HISTORY.append(LATEST_VELOCITY)
    
    latest_vel_avg = get_latest_velocity_xyzrpy()

    LEFT_RIGHT_MOVE_TRACKER.set_current_position(position)
    LEFT_RIGHT_MOVE_TRACKER.set_latest_imu_reading(latest_imu_message)
    LEFT_RIGHT_MOVE_TRACKER.set_current_velocity(latest_vel_avg)


def update_buoyancy_change_controller(current_action):
    BUOYANCY_CHANGE_TRACKER.set_goal_position(current_action.goal_pose)
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

    BUOYANCY_CHANGE_TRACKER.set_latest_imu_reading(latest_imu_message)
    BUOYANCY_CHANGE_TRACKER.set_current_position(robot_pose_xyzrpy)
    BUOYANCY_CHANGE_TRACKER.set_current_velocity(latest_vel_avg)


def update_open_loop_controller(current_action):
    CLOSED_LOOP_TRACKER.set_goal_position(current_action.goal_pose)
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


def get_effort_for_buoyancy_change(current_action):
    direction = current_action.ballast_change_direction
    tlevel = current_action.ballast_change_t_level
    min_effort_up = 0.05
    min_effort_down = 0.18

    if (direction > 0):
        max_effort = ((1.0 + min_effort_up) - tlevel) * config.MAX_EFFORT_BUOYANCY_CHANGE 
    if (direction < 0):
        max_effort = -1.0*max(tlevel, min_effort_down) * config.MAX_EFFORT_BUOYANCY_CHANGE 

    return max_effort


def act_on_current_action(current_action, pose_error):
    global ACTIVE_CONTROLLER, LAST_TRACKED_MARKER_TIME, SHOULD_CANCEL_HOLD

    goal_not_reached = current_action.reached_goal_time is None

    started = False
    if not current_action.is_started:
        tolerance = config.TIGHT_POSE_TOLERANCE
        #tolerance = config.COARSE_POSE_TOLERANCE
        reached_goal = all([abs(error) < tol for (error, tol) in zip(pose_error, tolerance)])

        if current_action.action_type == 'open_gripper':
            ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR
            if reached_goal:
                CLOSED_LOOP_TRACKER.y_i = config.DEFAULT_Y_I_GAIN
                CLOSED_LOOP_TRACKER.x_i = config.DEFAULT_X_I_GAIN

                if current_action.shift_left:
                    current_action.goal_pose[1] = current_action.goal_pose[1] - config.DROP_SHIFT_AMOUNT
                elif current_action.shift_right:
                    current_action.goal_pose[1] = current_action.goal_pose[1] + config.DROP_SHIFT_AMOUNT

                GRIPPER_HANDLER.start_opening_fingers()

                current_action.start()
                started = True

        elif current_action.action_type == 'bailing_release':
            ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR

            if reached_goal:
                current_action.gains_reset = False
                # push down by the amount in the command
                # set the gains on all axes to nothing
                current_action.goal_pose[2] = current_action.goal_pose[2] - current_action.thrust_down_amount

                if current_action.thrust_down_amount == 0.0:
                    CLOSE_LOOP_TRACKER.z_i = 0.0

                CLOSED_LOOP_TRACKER.x_i = 0.0
                CLOSED_LOOP_TRACKER.y_i = 0.0
                #CLOSED_LOOP_TRACKER.x_p = 0.1
                #CLOSED_LOOP_TRACKER.y_p = 0.1
                #CLOSED_LOOP_TRACKER.x_d = 0.1
                #CLOSED_LOOP_TRACKER.y_d = 0.1
                #CLOSED_LOOP_TRACKER.yaw_p = 0.1
                #CLOSED_LOOP_TRACKER.yaw_i = 0.1
                #CLOSED_LOOP_TRACKER.roll_i = 0.0
                #CLOSED_LOOP_TRACKER.yaw_d = 0.1
                CLOSED_LOOP_TRACKER.x_p = 0.0
                CLOSED_LOOP_TRACKER.x_d = 0.0
                CLOSED_LOOP_TRACKER.y_d = 0.0
                CLOSED_LOOP_TRACKER.yaw_p = 0.0
                CLOSED_LOOP_TRACKER.yaw_i = 0.0
                CLOSED_LOOP_TRACKER.roll_i = 0.0
                CLOSED_LOOP_TRACKER.yaw_d = 0.0

                if current_action.shift_left:
                    current_action.goal_pose[1] = current_action.goal_pose[1] - config.DROP_SHIFT_AMOUNT
                elif current_action.shift_right:
                    current_action.goal_pose[1] = current_action.goal_pose[1] + config.DROP_SHIFT_AMOUNT
                else:
                    CLOSED_LOOP_TRACKER.y_p = 0.0

                BALLAST_HANDLER.start_emptying_ballast_air(empty_time=current_action.bail_time)

                current_action.start()
                started = True
    
        elif current_action.action_type == 'left_right_move':
            robot_pose = get_robot_pose_xyzrpy()

            current_action.left_right_start_depth = robot_pose[2]
            current_action.left_right_start_yaw = robot_pose[5]

            goal_z = current_action.left_right_start_depth
            goal_yaw = current_action.left_right_start_yaw
            goal_x = current_action.goal_x
            goal_y = current_action.goal_y

            goal_pose = [
                goal_x,
                goal_y,
                goal_z,
                0.0,
                0.0,
                goal_yaw
            ]
            current_action.goal_pose = goal_pose
            rospy.loginfo("Starting left right move with goal pose {}".format(current_action.goal_pose))

            ACTIVE_CONTROLLER = LEFT_RIGHT_MOVE_CONTROL_SELECTOR
            LEFT_RIGHT_MOVE_TRACKER.clear_error_integrals()

            current_action.start()
            started = True

        elif current_action.action_type == 'close_gripper':
            ACTIVE_CONTROLLER = PID_CONTROL_SELECTOR
            if reached_goal:
                current_action.goal_pose[2] = current_action.goal_pose[2] - 0.40
                CLOSED_LOOP_TRACKER.z_i = 0.0
                CLOSED_LOOP_TRACKER.x_i = 0.0
                CLOSED_LOOP_TRACKER.y_i = 0.0
                #
                #CLOSED_LOOP_TRACKER.x_p = 0.1
                #CLOSED_LOOP_TRACKER.y_p = 0.1
                #CLOSED_LOOP_TRACKER.x_d = 0.1
                #CLOSED_LOOP_TRACKER.y_d = 0.1
                #CLOSED_LOOP_TRACKER.yaw_p = 0.1
                #CLOSED_LOOP_TRACKER.yaw_i = 0.1
                CLOSED_LOOP_TRACKER.x_p = 0.0
                CLOSED_LOOP_TRACKER.y_p = 0.0
                CLOSED_LOOP_TRACKER.x_d = 0.0
                CLOSED_LOOP_TRACKER.y_d = 0.0
                CLOSED_LOOP_TRACKER.yaw_p = 0.0
                CLOSED_LOOP_TRACKER.yaw_i = 0.0
                CLOSED_LOOP_TRACKER.roll_i = 0.0
                CLOSED_LOOP_TRACKER.yaw_d = 0.0
                started = True
                GRIPPER_HANDLER.start_closing_fingers()
                current_action.start()

            if DISPLAY is not None:
                DISPLAY.update_led_state([255,125,0],1)

        elif current_action.action_type == 'inflate_ballast':
            current_action.ballast_handler.start_filling_ballast_with_air()
            current_action.start()
            started = True

        elif current_action.action_type == 'deflate_ballast':
            current_action.ballast_handler.start_emptying_ballast_air()
            current_action.start()
            started = True

        elif current_action.action_type == 'change_platforms':
            config.TRACKED_MARKER_ID = current_action.to_platform_id
            ACTIVE_CONTROLLER = OPEN_LOOP_CONTROL_SELECTOR
            LAST_TRACKED_MARKER_TIME = None
            CLOSED_LOOP_TRACKER.clear_error_integrals()
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

        elif current_action.action_type == 'hold':
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

        elif current_action.action_type == 'change_buoyancy':
            ACTIVE_CONTROLLER = BUOYANCY_CHANGE_CONTROL_SELECTOR
            CLOSED_LOOP_TRACKER.clear_error_integrals()
            BUOYANCY_CHANGE_TRACKER.clear_error_integrals()

            max_effort = get_effort_for_buoyancy_change(current_action)

            BUOYANCY_CHANGE_TRACKER.set_z_override(
                max_effort
            )

            if current_action.ballast_change_direction > 0:
                #GRIPPER_HANDLER.start_closing_fingers()
                BALLAST_HANDLER.start_pulsing(
                    state=BALLAST_HANDLER.state_fill_with_air,
                    neutral_time=config.BALLAST_AIR_IN_PULSE_OFF_TIME,
                    on_time=config.BALLAST_AIR_IN_PULSE_TIME
                )
            else:
                BALLAST_HANDLER.start_pulsing(
                    state=BALLAST_HANDLER.state_empty_ballast_air,
                    neutral_time=config.BALLAST_AIR_OUT_PULSE_OFF_TIME,
                    on_time=config.BALLAST_AIR_OUT_PULSE_TIME
                )

            current_action.start()
            started = True
        else:
            raise Exception("Unrecognized action type!")

        if current_action.is_started:
            rospy.loginfo("Starting action: {}".format(current_action))
    else:
        if SHOULD_CANCEL_HOLD and current_action.action_type == 'hold':
            current_action.forced_complete = True
            SHOULD_CANCEL_HOLD = False

        if not current_action.is_complete(pose_error, last_tracked_marker_time=LAST_TRACKED_MARKER_TIME):
            if current_action.action_type == 'bailing_release':
                if (rospy.Time.now() - current_action.start_time).to_sec() > current_action.pre_open_time:
                    if not GRIPPER_HANDLER.is_opening:
                        GRIPPER_HANDLER.start_opening_fingers()
                    elif (rospy.Time.now() - GRIPPER_HANDLER.motion_started_time).to_sec() > 4.0 and not current_action.gains_reset:
                        current_action.gains_reset = True
                        CLOSED_LOOP_TRACKER.clear_error_integrals()
                        CLOSED_LOOP_TRACKER.z_p = DEFAULT_Z_P
                        CLOSED_LOOP_TRACKER.z_i = config.DEFAULT_Z_I_GAIN
                        CLOSED_LOOP_TRACKER.x_i =   pid_gains_dict['x_i']
                        CLOSED_LOOP_TRACKER.y_i =   pid_gains_dict['y_i']
                        CLOSED_LOOP_TRACKER.x_p =   pid_gains_dict['x_p']
                        CLOSED_LOOP_TRACKER.y_p =   pid_gains_dict['y_p']
                        CLOSED_LOOP_TRACKER.x_d =   pid_gains_dict['x_d']
                        CLOSED_LOOP_TRACKER.y_d =   pid_gains_dict['y_d']
                        CLOSED_LOOP_TRACKER.yaw_p = pid_gains_dict['yaw_p']
                        CLOSED_LOOP_TRACKER.yaw_i = pid_gains_dict['yaw_i']
                        CLOSED_LOOP_TRACKER.roll_i = pid_gains_dict['roll_i']
                        CLOSED_LOOP_TRACKER.yaw_d = pid_gains_dict['yaw_d']
                        current_action.goal_pose[2] = current_action.goal_pose[2] + current_action.thrust_down_amount + 0.1
                        rospy.loginfo("Setting goal pose to {} for after block release".format(current_action.goal_pose))
                        CLOSED_LOOP_TRACKER.clear_error_integrals()

        if current_action.is_complete(pose_error, last_tracked_marker_time=LAST_TRACKED_MARKER_TIME):
            rospy.loginfo("Completed action: {}. Error: {}. Tolerance {}".format(current_action, pose_error, current_action.pose_tolerance))

            if current_action.action_type == 'change_buoyancy':
                BALLAST_HANDLER.stop_pulsing()
                max_eff = get_effort_for_buoyancy_change(current_action)
                CLOSED_LOOP_TRACKER.clear_error_integrals(prev_buoyancy_input=max_eff)

            if current_action.action_type == 'open_gripper':
                CLOSED_LOOP_TRACKER.z_p = DEFAULT_Z_P
                CLOSED_LOOP_TRACKER.z_i = config.DEFAULT_Z_I_GAIN
                CLOSED_LOOP_TRACKER.x_p =   pid_gains_dict['x_p']
                CLOSED_LOOP_TRACKER.y_p =   pid_gains_dict['y_p']
                CLOSED_LOOP_TRACKER.x_d =   pid_gains_dict['x_d']
                CLOSED_LOOP_TRACKER.y_d =   pid_gains_dict['y_d']
                CLOSED_LOOP_TRACKER.yaw_p = pid_gains_dict['yaw_p']
                CLOSED_LOOP_TRACKER.yaw_i = pid_gains_dict['yaw_i']
                CLOSED_LOOP_TRACKER.yaw_d = pid_gains_dict['yaw_d']
                CLOSED_LOOP_TRACKER.clear_error_integrals()

            if current_action.action_type == 'close_gripper':
                CLOSED_LOOP_TRACKER.z_p = DEFAULT_Z_P
                CLOSED_LOOP_TRACKER.z_i = config.DEFAULT_Z_I_GAIN
                CLOSED_LOOP_TRACKER.x_i =   pid_gains_dict['x_i']
                CLOSED_LOOP_TRACKER.y_i =   pid_gains_dict['y_i']
                CLOSED_LOOP_TRACKER.x_p =   pid_gains_dict['x_p']
                CLOSED_LOOP_TRACKER.y_p =   pid_gains_dict['y_p']
                CLOSED_LOOP_TRACKER.x_d =   pid_gains_dict['x_d']
                CLOSED_LOOP_TRACKER.y_d =   pid_gains_dict['y_d']
                CLOSED_LOOP_TRACKER.yaw_p = pid_gains_dict['yaw_p']
                CLOSED_LOOP_TRACKER.yaw_i = pid_gains_dict['yaw_i']
                CLOSED_LOOP_TRACKER.roll_i = pid_gains_dict['roll_i']
                CLOSED_LOOP_TRACKER.yaw_d = pid_gains_dict['yaw_d']
                CLOSED_LOOP_TRACKER.clear_error_integrals()

            if current_action.action_type == 'bailing_release':
                CLOSED_LOOP_TRACKER.z_p = DEFAULT_Z_P
                CLOSED_LOOP_TRACKER.z_i = config.DEFAULT_Z_I_GAIN
                CLOSED_LOOP_TRACKER.x_i =   pid_gains_dict['x_i']
                CLOSED_LOOP_TRACKER.y_i =   pid_gains_dict['y_i']
                CLOSED_LOOP_TRACKER.x_p =   pid_gains_dict['x_p']
                CLOSED_LOOP_TRACKER.y_p =   pid_gains_dict['y_p']
                CLOSED_LOOP_TRACKER.x_d =   pid_gains_dict['x_d']
                CLOSED_LOOP_TRACKER.y_d =   pid_gains_dict['y_d']
                CLOSED_LOOP_TRACKER.yaw_p = pid_gains_dict['yaw_p']
                CLOSED_LOOP_TRACKER.yaw_i = pid_gains_dict['yaw_i']
                CLOSED_LOOP_TRACKER.roll_i = pid_gains_dict['roll_i']
                CLOSED_LOOP_TRACKER.yaw_d = pid_gains_dict['yaw_d']
                CLOSED_LOOP_TRACKER.clear_error_integrals()

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
    elif ACTIVE_CONTROLLER == BUOYANCY_CHANGE_CONTROL_SELECTOR:
        update_buoyancy_change_controller(current_action)
    elif ACTIVE_CONTROLLER == LEFT_RIGHT_MOVE_CONTROL_SELECTOR:
        update_left_right_move_controller(current_action)
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
        #GRIPPER_HANDLER.start_opening_fingers()

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
        elif ACTIVE_CONTROLLER == BUOYANCY_CHANGE_CONTROL_SELECTOR:
            pose_error = BUOYANCY_CHANGE_TRACKER.get_error()
        elif ACTIVE_CONTROLLER == LEFT_RIGHT_MOVE_CONTROL_SELECTOR:
            pose_error = LEFT_RIGHT_MOVE_TRACKER.get_error()
        else:
            raise Exception("Unrecognized active control selector!")
        if DISPLAY is not None:
            if LAST_ERROR_DISPLAY is None or (rospy.Time.now() - LAST_ERROR_DISPLAY).to_sec() > ERROR_DISPLAY_RATE:
                DISPLAY.update_lcd_display("{} / {}".format(current_action.sequence_id, number_actions), "{:0.1f} {:0.1f} {:0.2f}".format(*pose_error[0:3]))
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
        elif ACTIVE_CONTROLLER == BUOYANCY_CHANGE_CONTROL_SELECTOR:
            go_message = BUOYANCY_CHANGE_TRACKER.get_next_rc_override()
        elif ACTIVE_CONTROLLER == LEFT_RIGHT_MOVE_CONTROL_SELECTOR:
            go_message = LEFT_RIGHT_MOVE_TRACKER.get_next_rc_override()
        else:
            raise Exception("Unrecognized active control selector!")

        GRIPPER_HANDLER.update()
        BALLAST_HANDLER.update()

        utils.terminate_if_unsafe(go_message, 400, DRY_RUN)

        if not DRY_RUN:
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

    light_flasher_subscriber = rospy.Subscriber(
        config.INPUT_LED_TOPIC,
        std_msgs.msg.Bool,
        light_flash_callback,
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

    rospy.loginfo("Actions to complete:")
    for idx, action in enumerate(ACTIONS):
        rospy.loginfo("    {}: {}".format(idx, action))

    rospy.loginfo("Pauing for display initialization...")
    time.sleep(10)
    if DISPLAY is not None:
        DISPLAY.update_led_state([0, 255, 0], 1)       

    rospy.loginfo("Waiting for enough marker data from marker {}....".format(config.TRACKED_MARKER_ID))

    if not DRY_RUN:
        BALLAST_HANDLER.start_emptying_ballast_air()
        wait_for_marker_data()
    else:
        while LATEST_MARKER_MESSAGE is None:
            pass

    rospy.loginfo("Got marker data, going!!")

    for action in ACTIONS:
        action.gripper_handler = GRIPPER_HANDLER
        action.ballast_handler = BALLAST_HANDLER
        
        if action.high_level_build_step is None:
            raise Exception("Action {} is not associated with a high level build step!".format(action))

    SET_TRACKED_BUNDLE_IDS_PROXY(
        tracked_bundle_ids=[config.TRACKED_MARKER_ID]
    )

    run_build_plan(
        rc_override_publisher
    )

    rospy.loginfo("Finished experiment!")

    if not DRY_RUN:
        GRIPPER_HANDLER.stop()
        BALLAST_HANDLER.start_emptying_ballast_air()


if __name__ == "__main__":
    main()
