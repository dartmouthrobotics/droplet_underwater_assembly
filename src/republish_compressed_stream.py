#! /usr/bin/python

"""
Republishes a compressed stream into a raw image stream. Useful for debugging from information on bag files.
"""
import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2
import cv_bridge
import numpy as np


raw_image_publisher = None
bridge = None


def compressed_image_callback(compressed_image):
    np_arr = np.fromstring(compressed_image.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    output_message = bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
    raw_image_publisher.publish(output_message)



def initialize_compressed_republish_node():
    global raw_image_publisher, bridge
    rospy.init_node('compressed_image_republisher')

    compressed_image_in_topic = rospy.get_param("~compressed_image_in_topic")
    output_topic = rospy.get_param("~out_topic")

    rospy.Subscriber(compressed_image_in_topic, CompressedImage, compressed_image_callback)
    raw_image_publisher = rospy.Publisher(output_topic, Image, queue_size=1)

    bridge = cv_bridge.CvBridge()
    rospy.spin()


if __name__ == "__main__":
    initialize_compressed_republish_node()
