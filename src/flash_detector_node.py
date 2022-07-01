#! /usr/bin/python
import rospy
import cv2
from std_msgs.msg import Bool
import cv_bridge
from sensor_msgs.msg import Image, CompressedImage

# config stuff
compressed = True
flash_on_topic = "/flash_detected"

camera_topic = "/camera_array/cam0/image_raw"

if compressed:
    camera_topic = "/camera_array/cam0/image_raw/compressed"

# flash look area stuff
flash_area = [[0, 1440], [0, 200]]
flash_intensity_min = 254
min_white_blob_area = 1150

# global stuff
bridge = cv_bridge.CvBridge()

flash_on_publisher = None

def image_callback(image_message):
    global bridge
    raw_image = None

    if compressed:
        raw_image = bridge.compressed_imgmsg_to_cv2(image_message)
    else:
        raw_image = bridge.imgmsg_to_cv2(image_message)

    cropped_image = raw_image[
        flash_area[1][0]:flash_area[1][1],
        flash_area[0][0]:flash_area[0][1]
    ]
    #cv2.imshow('im', cropped_image)
    #cv2.waitKey(5)

    _, thresholded = cv2.threshold(cropped_image, flash_intensity_min, 255, cv2.THRESH_BINARY)

    _, cnts, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #cv2.drawContours(cropped_image, cnts, -1, (255, 0, 255), 3)

    max_area = float("-inf")

    for c in cnts:
        area = cv2.contourArea(c)
        if area > max_area:
            max_area = area

    message = Bool()
    message.data = False

    if max_area > min_white_blob_area:
       message.data = True

    flash_on_publisher.publish(message)

    #cv2.imshow('im', cropped_image)
    #cv2.waitKey(0)

    # threshold 
    # crop
    # find contours
    # find

    # if the biggest contour area is larger than
    # x, say yes

def main():
    global flash_on_publisher

    rospy.init_node("flash_detector")

    image_subscriber = None

    if compressed:
        image_subscriber = rospy.Subscriber(
            camera_topic,
            CompressedImage,
            image_callback,
            queue_size=1
        )
    else:
        image_subscriber = rospy.Subscriber(
            camera_topic,
            Image,
            image_callback,
            queue_size=1
        )

    flash_on_publisher = rospy.Publisher(
        flash_on_topic,
        Bool,
        queue_size=1
    )

    rospy.spin()

if __name__ == '__main__':
    main()
