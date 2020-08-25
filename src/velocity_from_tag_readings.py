#! /usr/bin/python2

import rospy
import stag_ros.msg
import tf.transformations
import utils

previous_message = None
runtime = 30.0

velocities = []
axis = 'yaw'
tracked_marker = 0

angular_axes = ['roll', 'pitch', 'yaw']

def distance(target, source, axis):
    if axis in angular_axes:
        return utils.angular_error_rads(source, target)

    return target - source

def get_position(marker_message, axis, marker_id):
    for marker in marker_message.markers:
        if marker.id == tracked_marker:
            marker_orientation_euler = tf.transformations.euler_from_quaternion(
                [
                    marker.pose.pose.orientation.x,        
                    marker.pose.pose.orientation.y,        
                    marker.pose.pose.orientation.z,        
                    marker.pose.pose.orientation.w        
                ]
            )

            if axis == "x":
                return marker.pose.pose.position.x

            if axis == "y":
                return marker.pose.pose.position.y

            if axis == "z":
                return marker.pose.pose.position.z

            if axis == "roll":
                marker_orientation_euler[0]

            if axis == "pitch":
                marker_orientation_euler[1]

            if axis == "yaw":
                marker_orientation_euler[2]

    return None


def marker_callback(marker_message):
    global previous_message, velocities

    if previous_message is not None:
        previous_pos = get_position(previous_message, axis, tracked_marker)
        current_pos = get_position(marker_message, axis, tracked_marker)

        if previous_pos is not None and current_pos is not None:
            velocities.append(
                distance(current_pos, previous_pos, axis) / (current_pos.header.stamp - previous_pos.header.stamp).to_sec() 
            )
        else:
            print "missed frame"
    
    previous_message = marker_message

def main():
    rospy.init_node("velocity_watcher")

    loop_rate = rospy.Rate(10)
    total_elapsed = 0.0
    start_time = rospy.Time.now()

    marker_subscriber = rospy.Subscriber(
        "/bluerov_controller/ar_tag_detector",
        stag_ros.msg.StagMarkers,
        marker_callback
    )

    while not rospy.is_shutdown() and total_elapsed <= runtime:

        total_elapsed = (rospy.Time.now() - start_time).to_sec()
        loop_rate.sleep()

    print(",".join(map(str, velocities)))

main()
