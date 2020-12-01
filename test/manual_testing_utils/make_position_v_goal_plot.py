import rospy
import tf
import copy

import mavros_msgs.msg
import geometry_msgs.msg
import math
import numpy
import stag_ros.msg

import matplotlib.pyplot

TRACKED_MARKER_ID = 0

current_pose = None
goal_pose = None
gripper_state = 0

def to_xyzrpy(translation, orientation):
    return numpy.array([
        translation[0],
        translation[1],
        translation[2],
        orientation[0],
        orientation[1],
        orientation[2]
    ])


def get_robot_pose_from_marker(marker):
    marker_orientation_simplifier_matrix = tf.transformations.euler_matrix(-math.pi / 2.0, math.pi / 2.0, 0.0)

    marker_pose_base_link = tf.transformations.concatenate_matrices(
       tf.transformations.translation_matrix([
           marker.pose.pose.position.x,
           marker.pose.pose.position.y,
           marker.pose.pose.position.z
       ]),
       tf.transformations.concatenate_matrices(
           tf.transformations.quaternion_matrix([
               marker.pose.pose.orientation.x,
               marker.pose.pose.orientation.y,
               marker.pose.pose.orientation.z,
               marker.pose.pose.orientation.w,
           ]),
           marker_orientation_simplifier_matrix,
       )
    )

    current_pose_matrix = tf.transformations.inverse_matrix(marker_pose_base_link)
    _, _, current_orientation, current_translation, _ = tf.transformations.decompose_matrix(current_pose_matrix)

    return current_translation, current_orientation


def marker_callback(marker_message):
    global current_pose
    for marker in marker_message.markers:
        if marker.id == TRACKED_MARKER_ID:
            current_pose = to_xyzrpy(*get_robot_pose_from_marker(marker))

num_in_state = 0

# could count number of open messages
def rc_override_callback(override_message):
    gripper_value = override_message.channels[8]
    global gripper_state, num_in_state

    if gripper_value > 1500:
        if gripper_state != 1:
            num_in_state = 0
            #print "open start"

        num_in_state = num_in_state + 1
        gripper_state = 1

        if num_in_state == 20:
            print "drop_at,{},{},{},{}".format(current_pose[0], current_pose[1], current_pose[2], current_pose[5])
            print "goal,{},{},{}".format(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z)

    elif gripper_value < 1500:
        if gripper_state != -1:
            num_in_state = 0
            #print "close start"

        num_in_state = num_in_state + 1
        gripper_state = -1
    else:
        if gripper_state != 0:
            #print "done"
            pass
        gripper_state = 0


def goal_pose_callback(goal_pose_message):
    global goal_pose
    goal_pose = goal_pose_message


# for the plot, will need to record the position half a second after a gripper open starts for every trial
# need to output that. I'll just print it and copy paste it where it needs to go I guess.

def main():
    rospy.init_node("goal_pose_vs_position_plot_maker")

    marker_subscriber = rospy.Subscriber("/bluerov_controller/ar_tag_detector", stag_ros.msg.StagMarkers, marker_callback)
    # subscribe to goal position

    goal_pose_subscriber = rospy.Subscriber("/goal_pose", geometry_msgs.msg.PoseStamped, goal_pose_callback)
    # watch rc override

    rc_override_subscriber = rospy.Subscriber("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn, rc_override_callback)
    # subscribe to tag readings. Need to do the tforms on the tag readings.

    sample_rate = rospy.Rate(45)

    pose_history = []
    goal_history = []
    gripper_history = []

    while not rospy.is_shutdown():
        if gripper_state is not None and goal_pose is not None and current_pose is not None:
            pose_history.append(copy.deepcopy(current_pose))
            goal_history.append(copy.deepcopy(goal_pose))
            gripper_history.append(copy.deepcopy(gripper_state))

        sample_rate.sleep()

    x_data = [pose[0][0] for pose in pose_history]
    g_data = [goal.pose.position.x for goal in goal_history]

    open_indices = []
    close_indices = []

    fig, ax = matplotlib.pyplot.subplots()
    cur_state = 0
    cur_range = [0,0]
    for idx, state in enumerate(gripper_history):
        if cur_state != state:
            if cur_state != 0:
                cur_range[1] = idx
                if cur_state == 1:
                    open_indices.append(list(cur_range))
                else:
                    close_indices.append(list(cur_range))
            else:
                cur_range[0] = idx

        cur_state = state

    print "opens", open_indices
    print "closes", close_indices

    ax.plot(x_data, label="pose x")
    ax.plot(g_data, label="goal")

    text_offset = 0.05
    for open_locs in open_indices:
        ax.annotate("Open", xy=(open_locs[0], x_data[open_locs[0]]), arrowprops=dict(arrowstyle="->"), xytext=(open_locs[0], x_data[open_locs[0]] + text_offset))
        ax.annotate("Done", xy=(open_locs[1], x_data[open_locs[1]]), arrowprops=dict(arrowstyle="->"), xytext=(open_locs[1], x_data[open_locs[1]] + text_offset))

    for close_locs in close_indices:
        ax.annotate("Close", xy=(close_locs[0], x_data[close_locs[0]]), arrowprops=dict(arrowstyle="->"), xytext=(close_locs[0], x_data[close_locs[0]] + text_offset))
        ax.annotate("Done", xy=(close_locs[1], x_data[close_locs[1]]), arrowprops=dict(arrowstyle="->"), xytext=(close_locs[1], x_data[close_locs[1]] + text_offset))

    ax.legend()
    matplotlib.pyplot.show()

if __name__ == "__main__":
    main()
