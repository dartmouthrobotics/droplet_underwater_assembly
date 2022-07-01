#! /usr/bin/python

import rospy
from droplet_underwater_assembly_libs import utils
import stag_ros.msg
from matplotlib import pyplot


data_z = []
data_x = []
data_y = []

def marker_callback(marker_msg):
    global data

    if len(marker_msg.markers) > 0:
        robot_pose = utils.get_robot_pose_from_marker(marker_msg.markers[0])
        robot_pose_xyzrpy = utils.to_xyzrpy(*robot_pose)

        pose = "{ "
        for x in robot_pose_xyzrpy[:3]:
            pose = pose + "{:.2f} ".format(x)
        pose = pose + "}"
        print pose

        data_z.append(robot_pose_xyzrpy[2])
        data_y.append(robot_pose_xyzrpy[1])
        data_x.append(robot_pose_xyzrpy[0])

def smooth(data):
    result = []
    for i in range(1, len(data)):
        j = i - 1
        result.append((data[i] + data[j]) / 2.0)

    return result

def main():
    rospy.init_node("location_stream_printer")

    marker_subscriber = rospy.Subscriber(
        "/bluerov_controller/ar_tag_detector",
        stag_ros.msg.StagMarkers,
        marker_callback,
        queue_size=1
    )

    while not rospy.is_shutdown():
        pass

    pyplot.plot(smooth(data_x), label="x") 
    pyplot.plot(smooth(data_y), label="y") 
    pyplot.plot(smooth(data_z), label="z") 
    pyplot.legend()
    pyplot.show()

if __name__ == '__main__':
    main()
