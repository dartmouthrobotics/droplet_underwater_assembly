#! /usr/bin/python
import rospy
import csv
import argparse
import distutils.util
import stag_ros.msg
from droplet_underwater_assembly_libs import utils

output_file = None
output_datapoint = None

TRACKED_MARKER_ID = 3

POSE_HISTORY = []


def output_pose_data():
    with open(output_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['datapoint id', 'x (meters)', 'y (meters)', 'z (meters)', 'yaw (radians)', 'timestamp'])

        for pose in POSE_HISTORY:
            writer.writerow(
                [output_datapoint, pose[0], pose[1], pose[2], pose[5], pose[6]]
            )


def marker_callback(marker_message):
    global POSE_HISTORY, TRACKED_MARKER_ID

    for marker in marker_message.markers:
        if marker.id == TRACKED_MARKER_ID:
            last_marker_pose = list(utils.to_xyzrpy(*utils.get_robot_pose_from_marker(marker)))
            last_marker_pose.append(marker_message.header.stamp)
            POSE_HISTORY.append(last_marker_pose)

def main():
    global output_file, output_datapoint

    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("datapoint", help="id of the datapoint for this set of info")
    arg_parser.add_argument("outputdir", help="directory to place the data in")
    arg_parser.add_argument("raw", help="Is this a raw datastream?")

    args, unknown = arg_parser.parse_known_args()
    output_datapoint = args.datapoint
    output_file = args.outputdir + str(output_datapoint) + ".csv"

    rospy.init_node('stream_recorder')
    rate = rospy.Rate(10)

    if distutils.util.strtobool(args.raw):
        marker_topic = "/bluerov_controller/ar_tag_detector"
    else:
        marker_topic = "/bluerov_controller/ar_tag_detector_2"

    rospy.loginfo("Listening for loc data at {}".format(
        marker_topic
    ))
    marker_subscriber = rospy.Subscriber(marker_topic, stag_ros.msg.StagMarkers, marker_callback)

    while not rospy.is_shutdown():
        rate.sleep()

    rospy.loginfo("Outputting csv file {} with location data!".format(output_datapoint))
    output_pose_data()

if __name__ == "__main__":
    main()
