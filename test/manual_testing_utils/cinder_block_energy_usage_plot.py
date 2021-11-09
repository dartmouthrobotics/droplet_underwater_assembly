import rospy
import rosbag
from matplotlib import pyplot

bag_file = rosbag.Bag('/media/sam/storage/uw-assembly-experiments/heavy_manipulation/first_cinder_block_test_09_20_2021.bag')


current_readings = []

for topic, message, time in bag_file.read_messages(topics='/mavros/battery'):
    current_readings.append((time, -1 * message.current))

first_time = current_readings[0][0].secs

pyplot.plot([c[0].secs - first_time for c in current_readings], [current_reading[1] for current_reading in current_readings])
pyplot.title("Current usage to hold position (full ballasting)")
pyplot.ylabel("Amps")
pyplot.xlabel("Seconds")
pyplot.show()
