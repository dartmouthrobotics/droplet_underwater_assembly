#! /usr/bin/python
import droplet
import build_plan
import rospy
import build_platform
import json

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

"""
TODO:
    * Handling of non-level build platforms
    Handling (more gracefully) of more slots on the platform
"""

"""
To handle a non-level build platform, what can we start with? Lets get a bag file (after fixing the hand, maybe on Sunday.)
The way that the non-level platform would work is that we compute the platform's roll and pitch in world frame based on the imu's orientation and each tag reading.

How can we do this? I guess a running average would be the best since it should never change.
How does it influence the things we tell the robot to do?
"""

def main():
    global DRY_RUN, goal_pose_publisher, transform_broadcaster
    rospy.init_node("droplet_underwater_assembly")

    config_file = rospy.get_param("~config_file")


    try:
        dry_run = rospy.get_param("~dry_run")
    except:
        rospy.logwarn("dry_run parameter not provided. Defaulting to test mode.")

    if dry_run:
        rospy.loginfo("Running in test mode.")

    config = None
    with open(config_file) as config_file:
        config = json.loads(config_file.read())

    droplet = droplet.Droplet()

    build_platform = build_platform.BuildPlatform(config)
    build_platform.send_to_tf_tree(droplet.transform_broadcaster)

    rospy.sleep(1.0)
    build_plan = build_plan.BuildPlan(config, droplet.transform_listener)

    rospy.loginfo("Waiting for enough marker data to start build plan....")
    droplet.wait_for_marker_data()
    rospy.loginfo("Got marker data. Starting build plan!!")

    droplet.run_build_plan(build_plan)


if __name__ == "__main__":
    main()
