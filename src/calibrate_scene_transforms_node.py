#! /usr/bin/python

"""
This script calibrates the relationship between the world marker
and the platform marker. 

It also computes a quaternion that can be used to align the world marker with

Run this script twice to get the proper information. First, run it to get the transform
that corrects the orientation of the world marker, then rerun to get usable platform transforms
"""

import rospy
from tf import transformations
import stag_ros.msg
import math
import sensor_msgs.msg
from numpy import linalg
import numpy
import numpy.matlib as npm

from droplet_underwater_assembly_libs import utils

SHOW_PLOTS = True

# first pos is platform, other is world
desired_pair = (4, 0)

# we need tform platform platform to main tag
marker_topic = "/bluerov_controller/ar_tag_detector"
ahrs_topic = "/mini_ahrs_ros/imu"


# mutable global state things
AHRS_MESSAGES = []

PLATFORM_WORLD_TRANSFORM_SAMPLES = []
GRAV_TO_WORLD_TRANSFORM_SAMPLES = []

NUMBER_MESSAGES_PROCESSED = 0

DEBUG_DATA = [
    [
        'plat_to_world_x',
        'plat_to_world_y',
        'plat_to_world_z',
        'plat_to_world_roll',
        'plat_to_world_pitch',
        'plat_to_world_yaw',
        'nearest_ahrs_roll',
        'nearest_ahrs_pitch',
        'nearest_ahrs_yaw',
        'ncorners_platform',
        'ncorners_world',
    ]
]


def ahrs_callback(ahrs_message):
    AHRS_MESSAGES.append(ahrs_message)


def zero_yaw(quat):
    # roll and pitch switched to handle misalignment bt ahrs and base link
    r,p,_ = transformations.euler_from_quaternion(quat, axes='sxyz')
    return transformations.quaternion_from_euler(r, p, 0.0, axes='sxyz')


def get_transform_platform_to_world(platform_marker, world_marker):
    """
    given the reading of a platform and world frame, find tform from platform to world
    """

    base_link_platform_mat = utils.simplify_marker_pose(
        utils.get_marker_pose_matrix(
            platform_marker
        )
    )

    # matrix describing the robot's pose in the world frame with grav correction
    base_link_world_mat = utils.get_grav_aligned_simplified_pose_matrix(
        world_marker
    )
    
    transform_world_to_platform = transformations.concatenate_matrices(
        base_link_world_mat,
        base_link_platform_mat
    )

    transform_platform_to_world = transformations.inverse_matrix(
        transform_world_to_platform
    )
    
    return transform_platform_to_world


def get_closest_ahrs_reading(timestamp):
    nearest_ahrs = None
    nearest_dist = float("inf")
    nearest_idx = 0

    for i, reading in enumerate(AHRS_MESSAGES):
        dist = abs((reading.header.stamp - timestamp).to_sec())

        if dist < nearest_dist:
            nearest_dist = dist
            nearest_ahrs = reading
            nearest_idx = i

    if nearest_idx > 0:
        del AHRS_MESSAGES[:nearest_idx-1]

    return nearest_ahrs


def get_orientation_ahrs(ahrs_message):
    return [
        ahrs_message.orientation.x,
        ahrs_message.orientation.y,
        ahrs_message.orientation.z,
        ahrs_message.orientation.w,
    ]


def get_orientation_marker(marker_message):
    return [
        marker_message.pose.pose.orientation.x,
        marker_message.pose.pose.orientation.y,
        marker_message.pose.pose.orientation.z,
        marker_message.pose.pose.orientation.w,
    ]


def strip_translation(pose_matrix):
    _, _, euler_angles, _, _ = transformations.decompose_matrix(pose_matrix)
    return transformations.euler_matrix(*euler_angles)


def get_world_orientation_in_grav(marker_message, ahrs_message):

    # goal: compute transform from grav to world but only the orientation part
    # returns a quaternion that transforms from grav to world frame
    ahrs_orientation = get_orientation_ahrs(ahrs_message)

    # negate the pitch
    r, p, y = transformations.euler_from_quaternion(ahrs_orientation, 'sxyz')
    ahrs_orientation = transformations.quaternion_from_euler(r, -p, y, 'sxyz')

    # transform grav to base link
    ahrs_orientation_matrix = transformations.quaternion_matrix(
        ahrs_orientation
    )

    # transform base link to world
    base_link_to_world = utils.simplify_marker_pose(
        utils.get_marker_pose_matrix(
            marker_message
        )
    )

    base_link_to_world_orientation = strip_translation(base_link_to_world)

    world_orientation_grav = transformations.concatenate_matrices(
        ahrs_orientation_matrix,
        base_link_to_world_orientation
    )

    if NUMBER_MESSAGES_PROCESSED % 120 == 0:
        rospy.loginfo("Grav debug stats")
        rospy.loginfo("ahrs orientation: {}".format(transformations.euler_from_quaternion(ahrs_orientation)))
        rospy.loginfo("base link world orientation: {}".format(transformations.decompose_matrix(base_link_to_world_orientation)[2]))
        rospy.loginfo("World orientation grav: {}".format(transformations.decompose_matrix(world_orientation_grav)[2]))

    return zero_yaw(
        transformations.quaternion_from_matrix(
            world_orientation_grav
        )
    )


def is_full_reading(marker_message):
    return len(marker_message.corners) >= 12


def marker_callback(message):
    global NUMBER_MESSAGES_PROCESSED 
    NUMBER_MESSAGES_PROCESSED = NUMBER_MESSAGES_PROCESSED + 1
    visible_markers = set([marker.id for marker in message.markers])

    platform_marker_id = desired_pair[0]
    world_marker_id = desired_pair[1]

    nearest_ahrs = get_closest_ahrs_reading(message.header.stamp)
    world_marker = None
    platform_marker = None

    if world_marker_id in visible_markers:
        world_marker = next(marker for marker in message.markers if marker.id == world_marker_id)

        if is_full_reading(world_marker):
            if nearest_ahrs is not None:
                GRAV_TO_WORLD_TRANSFORM_SAMPLES.append(
                    get_world_orientation_in_grav(
                        marker_message=world_marker,
                        ahrs_message=nearest_ahrs
                    )
                )
            
    if platform_marker_id in visible_markers:
        platform_marker = next(marker for marker in message.markers if marker.id == platform_marker_id)

    if platform_marker_id in visible_markers and world_marker_id in visible_markers:
        if is_full_reading(world_marker) and is_full_reading(platform_marker):
            transform = get_transform_platform_to_world(
                platform_marker=platform_marker,
                world_marker=world_marker
            )
            PLATFORM_WORLD_TRANSFORM_SAMPLES.append(
                transform
            )

            if SHOW_PLOTS:
                _, _, e, t, _ = transformations.decompose_matrix(transform)

                try:
                    e_ahrs = transformations.euler_from_quaternion(get_orientation_ahrs(nearest_ahrs))
                    DEBUG_DATA.append(
                        [t[0], t[1], t[2], e[0], e[1], e[2], e_ahrs[0], e_ahrs[1], e_ahrs[2], len(platform_marker.corners), len(world_marker.corners)]
                    )
                except Exception as e:
                    print e

        else:
            rospy.loginfo("Disregarding partial read of marker bundles")


def get_quaternion_and_translation_from_mat(transform_mat):
    _, _, euler_angles, translation, _ = transformations.decompose_matrix(
        transform_mat
    )

    orientation_quat = transformations.quaternion_from_euler(
        *euler_angles
    )

    return orientation_quat, translation


def get_average_transformation(transform_matrices):
    quaternions = []
    translations = []

    rotations_and_translations = map(get_quaternion_and_translation_from_mat, transform_matrices)

    for q, t in rotations_and_translations:
        quaternions.append(q)
        translations.append(t)

    average_orientation = get_average_rotation(quaternions)
    average_translation = get_average_translation(translations)

    return transformations.concatenate_matrices(
        transformations.translation_matrix(average_translation),
        transformations.quaternion_matrix(average_orientation)
    )


def get_average_translation(translation_samples):
    return list(
        sum(translation_samples) / float(len(translation_samples))
    )


def get_average_rotation(samples):
    rows = []

    for q in samples:
        rows.append(
            [q[3], q[0], q[1], q[2]]
        )

    arr = numpy.vstack(rows)

    result = averageQuaternions(arr)

    return [result[1], result[2], result[3], result[0]]


# from https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = numpy.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = numpy.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return numpy.real(eigenVectors[:,0].A1)


def save_platform_world_transform_data():
    with open("calibration_data_for_debug.csv", 'w') as f:
        out_lines = []

        for l in DEBUG_DATA:
            out_lines.append(
                ",".join(list(map(str, l)))
            )

        f.write("\n".join(out_lines))


def main():
    rospy.init_node("scene_transform_calibrator")

    marker_subscriber = rospy.Subscriber(
        marker_topic,
        stag_ros.msg.StagMarkers,
        marker_callback,
        queue_size=1
    )

    ahrs_subscriber = rospy.Subscriber(
        ahrs_topic,
        sensor_msgs.msg.Imu,
        ahrs_callback,
        queue_size=1
    )

    while not rospy.is_shutdown():
        pass

    if len(PLATFORM_WORLD_TRANSFORM_SAMPLES) > 0:
        rospy.loginfo("Got {} platform samples".format(len(PLATFORM_WORLD_TRANSFORM_SAMPLES)))
        average_transform = get_average_transformation(
            PLATFORM_WORLD_TRANSFORM_SAMPLES
        )

        rospy.loginfo("Tform platform {} to world {}".format(desired_pair[0], desired_pair[1]))

        rospy.loginfo("    Matrix {}".format(
            list(map(list, average_transform))
        ))

        _, _, euler, translation, _ = transformations.decompose_matrix(average_transform)

        rospy.loginfo("   Rotation (sxyz) {}".format(euler))
        rospy.loginfo("   Translation {}".format(translation))
        rospy.loginfo("   Distance {}".format(linalg.norm(translation)))

    grav_main_rectifier_quat = get_average_rotation(GRAV_TO_WORLD_TRANSFORM_SAMPLES)
    rospy.loginfo("Grav => marker id {} quat {} (no yaw)".format(desired_pair[1], list(grav_main_rectifier_quat)))
    rospy.loginfo("    euler (sxyz) {}".format(transformations.euler_from_quaternion(grav_main_rectifier_quat)))


if __name__ == '__main__':
    main()

    if SHOW_PLOTS:
        save_platform_world_transform_data()
