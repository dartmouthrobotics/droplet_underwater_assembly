import rospy
import rosbag
from matplotlib import pyplot

def avg(values):
    return sum(values) / float(len(values))

root_path = "/media/sam/storage/uw-assembly-experiments/heavy_manipulation/"

no_air_no_block_bag   = rosbag.Bag(root_path + "9_29_challenge_move_no_block_no_air.bag")
block_3_5_sec_air_bag = rosbag.Bag(root_path + "9_29_challenge_move_with_block_3_5_seconds_air.bag")
block_3_0_sec_air_bag = rosbag.Bag(root_path + "9_29_challenge_move_with_block_3_0_seconds_air.bag")
block_2_5_sec_air_bag = rosbag.Bag(root_path + "9_29_challenge_move_with_block_2_5_seconds_air.bag")
block_2_0_sec_air_bag = rosbag.Bag(root_path + "9_29_challenge_move_with_block_2_0_seconds_air.bag")

def get_energy_readings(bag_file):
    seq_id = -1
    action_type = "none"
    current_usages = []

    for topic, message, time in bag_file.read_messages(topics=["/mavros/battery", "/droplet_underwater_assembly/build_phase"]):
        if topic == "/mavros/battery":
            current_usages.append(
                (-1 * message.current, seq_id, time, action_type)
            )
        elif topic == "/droplet_underwater_assembly/build_phase":

            if message.action_sequence_id > seq_id:
                seq_id = message.action_sequence_id
                action_type = message.current_action_type

    return current_usages

no_block_readings = get_energy_readings(no_air_no_block_bag)
block_3_5 = get_energy_readings(block_3_5_sec_air_bag)
block_3_0 = get_energy_readings(block_3_0_sec_air_bag)
block_2_5 = get_energy_readings(block_2_5_sec_air_bag)
block_2_0 = get_energy_readings(block_2_0_sec_air_bag)

#pyplot.plot([i[0] for i in block_3_5], label='3.5 sec air')
#pyplot.plot([i[0] for i in block_3_0], label='3.0 sec air')
pyplot.title("Hold position -> Move 1m left -> hold position")
pyplot.plot([i[0] for i in block_3_5 if i[1] in [7, 8, 9]], label='3.5 sec air')
pyplot.plot([i[0] for i in block_2_5 if i[1] in [7, 8, 9]], label='2.5 sec air')
pyplot.plot([i[0] for i in block_2_0 if i[1] in [7, 8, 9]], label='2.0 sec air')
pyplot.xlabel("deciseconds")
pyplot.ylabel("Amps")

print("First hold average 2.0 sec {}".format(avg([i[0] for i in block_2_0 if i[1] == 7])))
print("First hold average 2.5 sec {}".format(avg([i[0] for i in block_2_5 if i[1] == 7])))
#pyplot.plot([i[0] for i in no_block_readings], label='no air no block')

pyplot.legend()
pyplot.show()
