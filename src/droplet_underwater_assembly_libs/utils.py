import rospy
import sys
import json
import numpy
import math
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet, CommandBool
import datetime
import mavros_msgs.msg
import matplotlib
import tf
import tf.transformations
import geometry_msgs.msg

from droplet_underwater_assembly_libs import config


ARMING_SERVICE_PROXY = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

def set_motor_arming(is_armed):
    if is_armed:
        rospy.loginfo("Arming motors")
    else:
        rospy.loginfo("Disarming motors")

    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        service_call_output = ARMING_SERVICE_PROXY(is_armed)
    except rospy.ServiceException:
        rospy.loginfo("Service call for motor arming failed!") 


def construct_stop_rc_message():
    stop_message = mavros_msgs.msg.OverrideRCIn()

    for i in range(len(stop_message.channels)):
        stop_message.channels[i] = 1500

    for i in range(len(stop_message.channels)):
        stop_message.channels[i] = 1500

    return stop_message


def construct_rc_message(channel_values):
    rc_message = construct_stop_rc_message() 

    for (index, value) in enumerate(channel_values):
        rc_message.channels[index] = value

    return rc_message


def ccw_angle_between_vectors(x1, y1, x2, y2):
    # from https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
    dot = x1*x2 + y1*y2      # dot product between [x1, y1] and [x2, y2]
    det = x1*y2 - y1*x2      # determinant
    angle = math.atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)

    if (angle < 0):
        return (2.0 * math.pi) + angle

    return angle


def angle_error_rads(from_angle, to_angle):
    # smallest distance in radians b/t ccw and cw angles
    x1 = math.cos(from_angle)
    y1 = math.sin(from_angle)

    x2 = math.cos(to_angle)
    y2 = math.sin(to_angle)

    ccw_distance = ccw_angle_between_vectors(x1, y1, x2, y2)

    if (ccw_distance > math.pi):
        # clockwise is faster in this case:
        cw_distance = -1 * ((2.0 * math.pi) - ccw_distance)
        return cw_distance

    return ccw_distance


def get_error(current_pose_xyzrpy, target_pose_xyzrpy):
    return [
        target_pose_xyzrpy[0] - current_pose_xyzrpy[0],
        target_pose_xyzrpy[1] - current_pose_xyzrpy[1],
        target_pose_xyzrpy[2] - current_pose_xyzrpy[2],
        angle_error_rads(current_pose_xyzrpy[3], target_pose_xyzrpy[3]),
        angle_error_rads(current_pose_xyzrpy[4], target_pose_xyzrpy[4]),
        angle_error_rads(current_pose_xyzrpy[5], target_pose_xyzrpy[5])
    ]


def matrix_to_pose_stamped(pose_matrix, parent_frame):
    _, _, orientation_euler, translation, _ = tf.transformations.decompose_matrix(pose_matrix)
    orientation_quaternion = tf.transformations.quaternion_from_euler(*orientation_euler)

    pose_stamped = geometry_msgs.msg.PoseStamped()

    pose_stamped.header.frame_id = parent_frame
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.pose.position.x = translation[0]
    pose_stamped.pose.position.y = translation[1]
    pose_stamped.pose.position.z = translation[2]

    pose_stamped.pose.orientation.x = orientation_quaternion[0]
    pose_stamped.pose.orientation.y = orientation_quaternion[1]
    pose_stamped.pose.orientation.z = orientation_quaternion[2]
    pose_stamped.pose.orientation.w = orientation_quaternion[3]

    return pose_stamped


def get_position_and_orientation_from_marker(marker):
    position = [
        marker.pose.pose.position.x,
        marker.pose.pose.position.y,
        marker.pose.pose.position.z,
    ]

    orientation = [
        marker.pose.pose.orientation.x,
        marker.pose.pose.orientation.y,
        marker.pose.pose.orientation.z,
        marker.pose.pose.orientation.w,
    ]

    return position, orientation


def get_marker_pose_matrix(marker_message):
    """
    accepts a marker message and returns a homogenous transformation matrix
    that describes that marker's pose in the message
    """
    marker_orientation_simplifier_matrix = tf.transformations.euler_matrix(
        -math.pi / 2.0,
        math.pi / 2.0, 
        0.0
    )

    position, orientation = get_position_and_orientation_from_marker(marker_message)
    position_mat = tf.transformations.translation_matrix(
        position
    )

    orientation_mat = tf.transformations.quaternion_matrix(
        orientation
    )

    return tf.transformations.concatenate_matrices(
        position_mat, tf.transformations.concatenate_matrices(
            orientation_mat,
            marker_orientation_simplifier_matrix
        )
    )


def simplify_marker_pose(pose_matrix):
    """
    accepts a homogenous transformation matrix that describes a marker in base link frame
    and rotates it so the inverse is a more intuitive coordinate frame.
    """
    return pose_matrix


def get_grav_aligned_inverse(marker_pose_mat):
    """
    applies grav alignment to the inverse of the marker pose. Used to get the robot's pose
    """
    correction_mat = tf.transformations.quaternion_matrix(
        config.GRAV_TO_WORLD_MARKER_QUATERNION
    )

    inverse_marker_pose = tf.transformations.inverse_matrix(marker_pose_mat)  

    return tf.transformations.concatenate_matrices(
        correction_mat,
        inverse_marker_pose
    )


def get_grav_aligned_simplified_pose_matrix(marker):
    simplified_grav_corrected_matrix = get_grav_aligned_inverse(
        simplify_marker_pose(
            get_marker_pose_matrix(
                marker
            )
        )
    )

    return simplified_grav_corrected_matrix


def get_robot_pose_from_marker(marker):
    """
    Given a marker reading for the world marker, returns the robot's pose
    in the format used by the rest of the code. Applies simplification 
    for coordinate system and gravity correction. Make sure to calibrate
    the grav correction!
    """
    simplified_grav_corrected_matrix = get_grav_aligned_simplified_pose_matrix(marker)

    _, _, current_orientation, current_translation, _ = tf.transformations.decompose_matrix(
        simplified_grav_corrected_matrix
    )

    return current_translation, current_orientation

    #marker_orientation_simplifier_matrix = tf.transformations.euler_matrix(
    #    -math.pi / 2.0,
    #    math.pi / 2.0, 
    #    0.0
    #)

    #marker_pose_base_link = tf.transformations.concatenate_matrices(
    #   tf.transformations.translation_matrix([
    #       marker.pose.pose.position.x,
    #       marker.pose.pose.position.y,
    #       marker.pose.pose.position.z
    #   ]),
    #   tf.transformations.concatenate_matrices(
    #       tf.transformations.quaternion_matrix([
    #           marker.pose.pose.orientation.x,
    #           marker.pose.pose.orientation.y,
    #           marker.pose.pose.orientation.z,
    #           marker.pose.pose.orientation.w,
    #       ]),
    #       marker_orientation_simplifier_matrix,
    #   )
    #)

    ## aligns roll and pitch with grav frame
    #correction_quat = config.GRAV_TO_WORLD_MARKER_QUATERNION
    #correction_mat = tf.transformations.quaternion_matrix(correction_quat)

    #base_link_in_marker_frame = tf.transformations.inverse_matrix(marker_pose_base_link)

    #current_pose_matrix = tf.transformations.concatenate_matrices(correction_mat, base_link_in_marker_frame)
    #_, _, current_orientation, current_translation, _ = tf.transformations.decompose_matrix(current_pose_matrix)

    #return current_translation, current_orientation


def to_xyzrpy(translation, orientation):
    return numpy.array([
        translation[0],
        translation[1],
        translation[2],
        orientation[0],
        orientation[1],
        orientation[2]
    ])


def pose_stamped_from_xyzrpy(xyzrpy, frame_id, seq, stamp):
    pose = geometry_msgs.msg.Pose()

    pose.position.x = xyzrpy[0]
    pose.position.y = xyzrpy[1]
    pose.position.z = xyzrpy[2]

    orientation = tf.transformations.quaternion_from_euler(xyzrpy[3], xyzrpy[4], xyzrpy[5], 'sxyz')

    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]

    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.seq = seq
    pose_stamped.header.stamp = stamp

    return pose_stamped


def send_transform_from_xyzrpy(transform_broadcaster, xyzrpy, parent_frame, child_frame, stamp):
    transform_broadcaster.sendTransform(
        tuple(xyzrpy[0:3]),
        tf.transformations.quaternion_from_euler(xyzrpy[3], xyzrpy[4], xyzrpy[5]),
        stamp,
        child_frame,
        parent_frame,
    )


def average_velocity(values_history, number_terms):
    total = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if number_terms > len(values_history):
        number_terms = len(values_history)
    
    if number_terms == 0:
        return total

    for i in range(number_terms):
        for j in range(6):
            total[j] = total[j] + values_history[len(values_history) - 1 - i][j]

    for j in range(6):
        total[j] = total[j] / float(number_terms)

    return total


def terminate_if_unsafe(message, max_pwm, dry_run):
    if any([abs(1500 - channel) > max_pwm for channel in message.channels]):
        rospy.logwarn("Too much pwm! Safety stopping on message: {}".format(message))

        if not DRY_RUN:
            set_motor_arming(False)

        sys.exit(1)
