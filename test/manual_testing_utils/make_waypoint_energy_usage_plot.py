import rospy
import rosbag
import itertools
from matplotlib import pyplot

def get_amp_hours_used(start, stop, current_readings):
    total = 0

    full_iter = iter(current_readings[start:stop])
    next(full_iter)
    prev_next = zip(iter(current_readings[start:stop-1]), full_iter)

    for current_reading, next_reading in prev_next:
        time_elapsed_hours = (next_reading[2] - current_reading[2]).to_sec() / 3600.0
        total = total + (current_reading[0] * 1000.0 * time_elapsed_hours) 

    return total
 

bag_file = rosbag.Bag('/media/sam/storage/uw-assembly-experiments/heavy_manipulation/moving_around_with_corrected_gains_no_block.bag')

seq_id = -1
current_usages = []

for topic, message, time in bag_file.read_messages(topics=["/mavros/battery", "/droplet_underwater_assembly/build_phase"]):
    if topic == "/mavros/battery":
        current_usages.append(
            (-1 * message.current, seq_id, time)
        )
    elif topic == "/droplet_underwater_assembly/build_phase":

        if message.action_sequence_id > seq_id:
            seq_id = message.action_sequence_id

changes = []

cur_seq = current_usages[0][1]
for idx, c in enumerate(current_usages):
    if c[1] != cur_seq:
        changes.append(idx)
        cur_seq = c[1]

for change in changes:
    pyplot.axvline(x=change, color='r', linestyle='--', label='Waypoint changed')

pyplot.plot([c[0] for c in current_usages], label="Current draw")
pyplot.title('Current usage for different waypoints')
pyplot.legend()

total_usage = get_amp_hours_used(0, len(current_usages), current_usages)
second_waypoint_usage = get_amp_hours_used(changes[1], changes[2], current_usages)

pairs = [(changes[0], changes[1]), (changes[1], changes[2]), (changes[2], changes[3]), (changes[3], changes[4]), (changes[4], len(current_usages))]

battery_cap = 15.6 * 1000.0 # mAh units (milli-amp hours)

for idx, pair in enumerate(pairs):
    used_energy = get_amp_hours_used(pair[0], pair[1], current_usages)
    print "from waypoint {} to waypoint {}: {} milli-amp hours | {}% of total {} mAh".format(idx, idx+1, used_energy, (used_energy / battery_cap)* 100.0, battery_cap)


pyplot.show()
