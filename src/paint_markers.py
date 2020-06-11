#! /usr/bin/python2
import rospy
import cv2
import subprocess
import numpy

from sensor_msgs.msg import CompressedImage

have_cam_info = False

camera_matrix = numpy.array([[977.570929, 0.000000, 734.708934], [0.000000, 976.560225, 571.300468], [0.000000, 0.000000, 1.000000]])
distortion_coefficients = numpy.array([-0.292717, 0.093499, 0.001386, -0.000104, 0.000000])

stag_log_dir = "/home/sam/Dev/ros-catkin-workspace/src/stag_ros/modules/stag/build/log/"

global edge_video_out
global marker_video_out


def image_callback(compressed_image):
    global edge_video_out, marker_video_out

    raw_data = numpy.fromstring(compressed_image.data, numpy.uint8)
    cv_image = cv2.imdecode(raw_data, cv2.IMREAD_GRAYSCALE)

    undistorted_image = cv2.undistort(cv_image, camera_matrix, distortion_coefficients)

    input_image_path = stag_log_dir + "input.png" 
    cv2.imwrite(input_image_path, cv_image)

    try:
        subprocess.check_output([stag_log_dir + "../stag_main", input_image_path, "21", "7"])
    except:
        pass

    marked_image = cv2.imread(stag_log_dir + "6 markers.png")
    print "Processed image"
    marker_video_out.write(marked_image)

    edge_image = cv2.imread(stag_log_dir + "1 edges.png")
    edge_video_out.write(edge_image)

    cv2.imshow("markers", marked_image)
    cv2.imshow("edges", edge_image)
    cv2.waitKey(5) # waits until a key is pressed


def main():
    global edge_video_out, marker_video_out
    print("Marker painter listening for camera images")
    edge_video_out = cv2.VideoWriter(
        'edge_video.avi',
        cv2.VideoWriter_fourcc('M','J','P','G'),
        5,
        (1440, 1080)
    )

    marker_video_out = cv2.VideoWriter(
        'marker_video.avi',
        cv2.VideoWriter_fourcc('M','J','P','G'),
        5,
        (1440, 1080)
    )
    rospy.init_node("marker_painter")
    rospy.Subscriber("/camera_array/cam0/image_raw/compressed", CompressedImage, image_callback, queue_size=10000)

    wait_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        wait_rate.sleep()

    edge_video_out.release()
    marker_video_out.release()

if __name__ == "__main__":
    main()
