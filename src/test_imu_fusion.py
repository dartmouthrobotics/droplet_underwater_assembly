import rospy

from sensor_msgs.msg import Imu
import tf

br = None


def imu_callback(imu_message):
    global br

    orientation = [
        imu_message.orientation.x,
        imu_message.orientation.y,
        imu_message.orientation.z,
        imu_message.orientation.w
    ]

    br.sendTransform((0,0,0), orientation, rospy.Time.now(), "/imu", "/world")


def main():
    global br
    rospy.init_node("imu_listener")

    imu_sub = rospy.Subscriber(
        "/imu/data",
        Imu,
        imu_callback
    )

    br = tf.TransformBroadcaster()

    rospy.spin()
    

main()
