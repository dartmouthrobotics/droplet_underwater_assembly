#! /usr/bin/python

# what do we need to do to make this less of a mess...
# these globals need to be moved somewhere...
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
import reporting
import trajectory_tracker
import assembly_action
import config
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


LATEST_MARKER_MESSAGE = None
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

debug_pose_publisher = None
latest_imu_message = None

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


ACTIONS = [
    # round 1
    assembly_action.AssemblyAction('move', config.OVER_BLOCK_1_POSE_LOW, config.TIGHT_POSE_TOLERANCE),
    assembly_action.AssemblyAction('close_gripper', config.OVER_BLOCK_1_POSE_LOW, config.COARSE_POSE_TOLERANCE),
    assembly_action.AssemblyAction('move', config.OVER_BLOCK_1_POSE_HIGH, config.COARSE_POSE_TOLERANCE),
    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
    assembly_action.AssemblyAction('move', config.OVER_BLOCK_1_POSE_HIGH, config.TIGHT_POSE_TOLERANCE),
    assembly_action.AssemblyAction('open_gripper', config.OVER_BLOCK_1_POSE_HIGH, config.TIGHT_POSE_TOLERANCE),
    assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.COARSE_POSE_TOLERANCE),
]

#ACTIONS = [
#    # round 1
#    AssemblyAction('move', OVER_BLOCK_1_POSE),
#]


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
    while ((datetime.datetime.now() - start_time).total_seconds() < float(config.EXPERIMENT_DURATION_SECONDS)) and not rospy.is_shutdown():
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
        time=config.EXPERIMENT_DURATION_SECONDS
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
