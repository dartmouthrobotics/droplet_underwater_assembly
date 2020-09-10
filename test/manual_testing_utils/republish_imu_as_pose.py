#! /usr/bin/python2

import rospy
import tf
import sensor_msgs.msg

broadcaster = None

def imu_callback(imu_message):
    orientation_quat = [
        imu_message.orientation.x,
        imu_message.orientation.y,
        imu_message.orientation.z,
        imu_message.orientation.w
    ]

    broadcaster.sendTransform((0.0, 0.0, 0.1), orientation_quat, rospy.Time.now(), "/base_link", "/world")

def main():
    global broadcaster

    rospy.init_node("imu_republisher")

    broadcaster = tf.TransformBroadcaster()
    subscriber = rospy.Subscriber("/mavros/imu/data", sensor_msgs.msg.Imu, imu_callback)

    rospy.spin()

main()
