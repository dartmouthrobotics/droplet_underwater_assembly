import rospy
import rosbag
from matplotlib import pyplot

air_times = [1.0, 1.5, 2.0]

directory = "/media/sam/storage/uw-assembly-experiments/heavy_manipulation/"

bag_files = [
    "10_26_2021_1_0_sec_air_18_mark_lateral.bag",
    "10_26_2021_1_5_sec_air_24_mark_lateral.bag",
    "10_26_2021_2_0_sec_air_all_mark_lateral.bag",
]


def get_energy_readings(bag_file):
    # returns (current, seq_id, time, action_type)
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

energy_by_air_time = {}

filter_air_times = [1.5, 1.0, 2.0]
for bag_file, air_time in zip(bag_files, air_times):
    if air_time in filter_air_times:
        bag = rosbag.Bag(directory + bag_file)
        energy_by_air_time[air_time] = get_energy_readings(bag)
        all_actions = set([i[1] for i in energy_by_air_time[air_time]])

filter_actions = [5, 6, 7, 8]
filter_actions = list(sorted(all_actions))[5:]
for air_time, energy in sorted(energy_by_air_time.items(), key=lambda x: x[0]):
    pyplot.plot(
        [i[0] for i in energy if i[1] in filter_actions],
        label="air time: {}".format(air_time)
    )

pyplot.title("Moving left and right")
pyplot.ylabel("Current (Amps)")
pyplot.xlabel("Tenths of seconds")
pyplot.legend()
pyplot.show()
