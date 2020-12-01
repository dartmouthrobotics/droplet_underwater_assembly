import rospy
import tf.transformations
import sensor_msgs.msg

imu_topic = "/mavros/imu/data"
outfile = "/home/sam/Dev/platform_online_calibration_tests/position_2_pixhawk.csv"
run_time = 100.0

def imu_callback(imu_message):
    roll, pitch, _ = tf.transformations.euler_from_quaternion([
        imu_message.orientation.x,
        imu_message.orientation.y,
        imu_message.orientation.z,
        imu_message.orientation.w
    ])

    with open(outfile, "a") as f:
        f.write("{},{}\n".format(roll, pitch))

def main():
    rospy.init_node("imu_reader")

    with open(outfile, "w") as f:
        f.write("roll,pitch\n")

    imu_subscriber = rospy.Subscriber(imu_topic, sensor_msgs.msg.Imu, imu_callback, queue_size=1000)

    start_time = rospy.Time.now()

    rate = rospy.Rate(10)

    while (rospy.Time.now() - start_time).to_sec() < run_time:
        rate.sleep()

        if rospy.is_shutdown():
            break

if __name__ == "__main__":
    main()
