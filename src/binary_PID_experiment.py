import rospy
import sys
import json
import numpy
import math
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet
import datetime
import mavros_msgs.msg
import ar_track_alvar_msgs.msg
import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot
from mavros_msgs.srv import CommandBool
import tf

ARMING_SERVICE_PROXY = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
CHANGE_PARAM_PROXY = rospy.ServiceProxy('/mavros/param/set', ParamSet)


YAW_RC_CHANNEL = 3
MARKER_ID = 1
CUTOFF = 0.00

pwm = abs(int(sys.argv[1]))
publish_duration_seconds = float(sys.argv[2])

next_yaw_pwm = 0

angle_history = []
pwm_history = []

collecting_data = False


def set_motor_arming(is_armed):
    if is_armed:
        print "Arming motors"
    else:
        print "Disarming motors"

    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        service_call_output = ARMING_SERVICE_PROXY(is_armed)
    except rospy.ServiceException:
        rospy.loginfo("Service call for motor arming failed!") 


def plot_data():
    global angle_history, pwm_history

    output_plot_prefix = "{pwm}_pwm_binary_p_control".format(pwm=pwm)
    matplotlib.pyplot.title("{} pwm binary P. {} seconds".format(pwm, float(publish_duration_seconds)))
    matplotlib.pyplot.plot(angle_history, label="tag angle")

    #matplotlib.pyplot.plot([i / pwm for i in pwm_history], label="published pwm")
    matplotlib.pyplot.legend()
    matplotlib.pyplot.savefig(output_plot_prefix + ".png")


def establish_ROS_control():
    myparam = ParamValue()

    # To have control from ROS
    rospy.wait_for_service('/mavros/param/set')
    try:
        myparam.integer = 255
        myparam.real = 0
        out = CHANGE_PARAM_PROXY("SYSID_MYGCS", myparam)
        if out.success:
            rospy.loginfo("ROS control esablished")
        else:
            rospy.loginfo("Failed gaining ROS control")
    except rospy.ServiceException:
        rospy.loginfo("Service call failed")


def marker_callback(marker_message):
    global angle_history, next_yaw_pwm, pwm_history, collecting_data, pwm

    marker_found = False
    for marker in marker_message.markers:
        if marker.id == MARKER_ID:
            marker_found = True
            marker_angle = math.atan2(marker.pose.pose.position.y, marker.pose.pose.position.x)

            if abs(marker_angle) < CUTOFF:
                next_yaw_pwm = 0
            else:
                if marker_angle > 0:
                    next_yaw_pwm = -1 * pwm
                else:
                    next_yaw_pwm = 1 * pwm

            if collecting_data:
                pwm_history.append(next_yaw_pwm)
                angle_history.append(marker_angle)

    if not marker_found:
        print "Marker not found. Setting next pwm to 0"
        next_yaw_pwm = 0


def run_binary_P_control_experiment(rc_override_publisher):
    global collecting_data, next_yaw_pwm

    publish_rate = rospy.Rate(30)
    go_message = mavros_msgs.msg.OverrideRCIn()
    stop_message = mavros_msgs.msg.OverrideRCIn()

    for i in range(len(go_message.channels)):
        go_message.channels[i] = 1500

    for i in range(len(stop_message.channels)):
        stop_message.channels[i] = 1500

    if abs(pwm) > 60:
        print "Too fast!!!!! PWM must be less than 60. You don't want to make a fountain in the lab do you?"
        sys.exit(1)

    establish_ROS_control()
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)

    set_motor_arming(True)
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)

    collecting_data = True
    print "Starting data collection for pwm {pwm}".format(pwm=pwm)
    start_time = datetime.datetime.now()
    while (datetime.datetime.now() - start_time).total_seconds() < float(publish_duration_seconds):
        go_message.channels[YAW_RC_CHANNEL] = 1500 + next_yaw_pwm 

        if any([abs(1500 - channel) > 50 for channel in go_message.channels]):
            print "Too much pwm!"
            set_motor_arming(False)
            sys.exit(1)

        rc_override_publisher.publish(go_message)
        publish_rate.sleep()

    print "Finished data collection"
    collecting_data = False
    set_motor_arming(False)
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)
    rc_override_publisher.publish(stop_message)


def main():
    rospy.init_node("binary_pid_control")
    marker_subscriber = rospy.Subscriber("/bluerov_controller/ar_tag_detector", ar_track_alvar_msgs.msg.AlvarMarkers, marker_callback)
    rc_override_publisher = rospy.Publisher("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn, queue_size=10)

    print "Running binary PID control experiment with pwm {pwm} and time {time}".format(pwm=pwm, time=publish_duration_seconds)
    run_binary_P_control_experiment(rc_override_publisher)

    print "Plotting data"
    plot_data()


if __name__ == "__main__":
    main()
