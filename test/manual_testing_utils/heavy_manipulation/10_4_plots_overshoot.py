import rosbag
import rospy

import rospy
import rosbag
from matplotlib import pyplot

air_times = [1.7, 1.8, 1.9, 2.1, 2.3, 2.5, 2.7, 0.0]

directory = "/media/sam/storage/uw-assembly-experiments/heavy_manipulation/"

bag_files = list(map(lambda x: rosbag.Bag(directory + x), [
    "10_04_trials_1_7_secs_air.bag",
    "10_04_trials_1_8_secs_air.bag",
    "10_04_trials_1_9_secs_air.bag",
    "10_04_trials_2_1_secs_air.bag",
    "10_04_trials_2_3_secs_air.bag",
    "10_04_trials_2_5_secs_air.bag",
    "10_04_trials_2_7_secs_air.bag",
    "10_04_trials_no_block.bag",
]))

def get_overshoots(bag_file):
    topics = ["/droplet_underwater_assembly/build_phase"]
    overshoots = []
    for topic, message, time in bag_file.read_messages(topics=topics):
        overshoots.append(
            (message.goal_location[1] - message.current_location[1], message.action_sequence_id, message.goal_location[1], message.move_tolerance[1])
        )

    return overshoots

error_by_air_time = {}
filter_air_times = [1.7, 1.9, 0.0]
for air_time, bag in zip(air_times, bag_files):
    if air_time in filter_air_times:
        error_by_air_time[air_time] = get_overshoots(bag)
        all_actions = set([i[1] for i in error_by_air_time[air_time]])

filter_actions = [9, 10, 11, 12, 13]
goals = set()
tols = set()
#filter_actions = list(sorted(all_actions))[5:]
for air_time, error in sorted(error_by_air_time.items(), key=lambda x: x[0]):
    label = "air time: {}".format(air_time)
    for e in error:
        if e[2] != -0.13:
            goals.add(e[2])

        tols.add(e[3])

    if air_time == 0.0:
        label = "No block"

    pyplot.plot(
        [i[0] for i in error if i[1] in filter_actions],
        label=label
    )

goals = list(goals)
tols = list(tols)

pyplot.axhline(-tols[0], color='r', ls='--', label="tolerance region")
pyplot.axhline(tols[0], color='r', ls="--")

pyplot.title("Moving left and right")
pyplot.ylabel("Error (meters)")
pyplot.xlabel("Fourtieths of seconds")
pyplot.legend()
pyplot.show()
    
