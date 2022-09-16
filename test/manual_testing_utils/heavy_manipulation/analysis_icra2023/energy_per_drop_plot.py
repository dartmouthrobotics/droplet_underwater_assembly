import rospy
import rosbag
import copy
import numpy as np
import numpy.linalg as linalg
import matplotlib.patches as patches
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

from matplotlib import pyplot

import bag_files

tight_tol_xy = 0.02
selected_file = bag_files.six_block_pyramid_success
selected_file = bag_files.three_block_column_success
selected_file = bag_files.five_block_pyramid_success
build_phase_topic = '/droplet_underwater_assembly/build_phase'

all_files = [
    bag_files.six_block_pyramid_success,
    bag_files.three_block_column_success,
    bag_files.five_block_pyramid_success,
]

def get_data_for_grasp_actions(bag_file_path, topics):
    bag_file = rosbag.Bag(bag_file_path)

    topic_extension = []

    if build_phase_topic not in topics:
        topic_extension = [build_phase_topic]

    in_manipulation_action = False
    prev_message_action = None

    release_actions = ['bailing_release', 'open_gripper']

    result = []
    current_messages_by_topic = {topic: [] for topic in topics}

    for topic, message, time in bag_file.read_messages(topics=topics + topic_extension):
        if topic == build_phase_topic:
            if prev_message_action != 'close_gripper' and message.current_action_type == 'close_gripper':
                in_manipulation_action = True

            if message.current_action_type not in release_actions and prev_message_action in release_actions:
                in_manipulation_action = False
                result.append(copy.deepcopy(current_messages_by_topic))
                current_messages_by_topic = {
                    topic: [] for topic in topics
                }

            prev_message_action = message.current_action_type

        if in_manipulation_action and topic in topics:
            current_messages_by_topic[topic].append(message)

    return result


def get_amp_hours_consumed(battery_messages):
    y_vals = [
        -m.current for m in battery_messages
    ]

    first_time = battery_messages[0].header.stamp

    x_vals = [
        (m.header.stamp - first_time).to_sec() / 3600.0 for m in battery_messages
    ]

    return np.trapz(y=y_vals, x=x_vals)
    
    #secs_per_hour = 3600.0
    ## just by naive rectangle method
    #prev_m = None

    #total = 0.0

    #for m in battery_messages:
    #    if prev_m is None:
    #        prev_m = m
    #        continue

    #    delta_t_hours = (m.header.stamp - prev_m.header.stamp).to_sec() / secs_per_hour
    #    prev_height = -prev_m.current

    #    total = total + (prev_height * delta_t_hours)
    #    prev_m = m

    #return total


def avg(data):
    return sum(data) / float(len(data))


def get_position_errors_during_drops(manip_segmented_data, drop_type):
    time_delta_after_open_start = 0.0
    if drop_type == 'open_gripper':
        time_delta_after_open_start = 5.2

    errors = []

    drop_locations = []
    for drop_data in manip_segmented_data:
        open_msgs = [m for m in drop_data[build_phase_topic] if m.current_action_type == drop_type]

        if open_msgs:
            start_time = open_msgs[0].header.stamp
            goal_pose = np.array(open_msgs[0].goal_location[0:2])

            min_delta = float("inf")
            min_idx = None

            for i, m in enumerate(open_msgs):
                delta = abs((m.header.stamp - start_time).to_sec() - time_delta_after_open_start)

                if delta < min_delta:
                    min_delta = delta
                    min_idx = int(i)

            nearest_message = open_msgs[min_idx]
            drop_location = np.array(nearest_message.current_location[0:2]) - goal_pose
            drop_locations.append(drop_location)

            error = linalg.norm(drop_location - goal_pose)
            errors.append(error)

    return drop_locations


def get_phase_name(s):
    if s == 'close_gripper':
        return 'Grasping block'
    elif s == 'change_buoyancy':
        return 'Adding buoyancy'
    elif s == 'bailing_release' or s == 'open_gripper':
        return 'Releasing block'
    elif s == 'hold' or s == 'move':
        return 'Transporting block'

    raise Exception(s)


def get_phase_color(name):
    if name == 'Grasping block':
        return (195.0 / 255.0, 231.0 / 255.0, 235.0 / 255.0)
    elif name == 'Adding buoyancy':
        return (158.0 / 255.0, 192.0 / 255.0, 237.0 / 255.0)
    elif name == 'Releasing block':
        return (125.0 / 255.0, 157.0 / 255.0, 221.0 / 255.0)
    elif name == 'Transporting block':
        return (116.0 / 255.0, 204.0 / 255.0, 211.0 / 255.0)


def get_phase_start_times(data, initial_phase):
    phase_start_times = [
        (initial_phase, 0.0)
    ]
    current_phase = initial_phase
    first_time = data['/droplet_underwater_assembly/build_phase'][0].header.stamp

    for message in data['/droplet_underwater_assembly/build_phase'][1:]:
        n = get_phase_name(message.current_action_type)
        if current_phase != n:
            phase_start_times.append(
                (n, (message.header.stamp - first_time).to_sec())
            )
            current_phase = n

    return phase_start_times


def make_energy_timeline_plot(data):
    # step 1 is get all the battery messages out
    energy_readings = []
    time_points = []
    first_time = None

    end_time = 0.0
    for batt_msg in data[0]['/mavros/battery']:
        if first_time is None:
            first_time = batt_msg.header.stamp

        energy_readings.append(-batt_msg.current)
        time_points.append((batt_msg.header.stamp - first_time).to_sec())
        end_time = (batt_msg.header.stamp - first_time).to_sec()

    initial_phase = get_phase_name(data[0]['/droplet_underwater_assembly/build_phase'][0].current_action_type)
    phase_start_times = get_phase_start_times(data[0], initial_phase)

    fig, ax = pyplot.subplots()

    t1 = phase_start_times[2][1]
    t2 = phase_start_times[3][1]
    current_ms = []

    for m in data[0]['/mavros/battery']:
        t = (m.header.stamp - first_time).to_sec()

        if t1 <= t <= t2:
            current_ms.append(-m.current)

    print("Average current for transit: {}".format(avg(current_ms)))


    last_phase = 0.0
    for i, (phase, time) in enumerate(phase_start_times):
        ax.axvline(x=time, c='r', linestyle='--')
        h = 3.0
        et = 0.0
        if i < len(phase_start_times)-1:
            et = phase_start_times[i+1][1]
        else:
            et = end_time

        ax.add_patch(patches.Rectangle((time, 0.0), width=(et - time), height=h, facecolor=get_phase_color(phase), label=phase))

        last_phase = time

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Current draw (amps)')
    #ax.set_title("Energy use while stacking a block")

    ax.plot(time_points, energy_readings, c='k')
    ax.legend()
    pyplot.savefig("./energy_use_timeline_for_block_stack.pdf")
    pyplot.show()


def make_grasp_alignment_plot(data, manip_number):
    close_actions = []
    for m in data[manip_number]['/droplet_underwater_assembly/build_phase']:
        if m.current_action_type == 'close_gripper':
            close_actions.append(m)

    fig, ax = pyplot.subplots()

    errs = []
    xerrs = []
    yerrs = []

    for m in close_actions:
        errs.append(
            linalg.norm(
                np.array([m.goal_location[0], m.goal_location[1]]) - np.array([m.current_location[0], m.current_location[1]])
            )
        )

        xerrs.append(m.goal_location[0] - m.current_location[0])
        yerrs.append(m.goal_location[1] - m.current_location[1])

    print(errs)
    ax.plot(errs)
    #ax.plot(yerrs)
    pyplot.show()


def get_phase_timing_stats(data):
    data_start = data[0]['/droplet_underwater_assembly/build_phase'][0].header.stamp
    data_end = data[-1]['/droplet_underwater_assembly/build_phase'][-1].header.stamp
    data_elapsed_time = (data_end - data_start).to_sec()

    all_batt_messages = []

    total_phase_times = {}

    manip_totals = []

    for manip in data:
        phase_starts = get_phase_start_times(manip, 'Grasping block')
        manip_start =  manip['/droplet_underwater_assembly/build_phase'][0].header.stamp
        manip_end =    manip['/droplet_underwater_assembly/build_phase'][-1].header.stamp
        manip_end_seconds = (manip_end - manip_start).to_sec()
        manip_totals.append(manip_end_seconds)

        batt_messages_by_phase = {}

        for i, (name, start_time) in enumerate(phase_starts):
            if name not in total_phase_times:
                total_phase_times[name] = 0.0

            if i < len(phase_starts)-1:
                end_time = phase_starts[i+1][1]
            else:
                end_time = manip_end_seconds

            total_phase_times[name] = total_phase_times[name] + end_time - start_time 

    time_in_manips = 0.0
    percent_total_check = 0.0

    print("---Time stats---")
    print("    Total time: {} seconds".format(data_elapsed_time))
    for entry in sorted(total_phase_times.keys()):
        total_time_in_phase = total_phase_times[entry]
        time_in_manips = time_in_manips + total_time_in_phase
        percentage = (total_time_in_phase * 100.0) / data_elapsed_time
        percent_total_check = percent_total_check + percentage

        print "    {}: {:.1f}".format(entry, percentage)

    time_returning_to_blocks = data_elapsed_time - time_in_manips
    return_to_block_percent = (time_returning_to_blocks * 100.0) / data_elapsed_time
    print "    {}: {:.1f}%".format("Moving to next grasp", return_to_block_percent)

    print("    Total: {}%".format(percent_total_check + return_to_block_percent))
    print("    Average time to manipulate: {}s".format(avg(manip_totals)))


vals = []
def get_energy_stats_by_phase(bag_file_path):
    global vals
    print("---Energy stats---")
    bag_file = rosbag.Bag(bag_file_path)
    topics = [
        '/droplet_underwater_assembly/build_phase',
        '/mavros/battery'
    ]
    previous_phase = None
    all_batt_messages = []

    for topic, message, rostime in bag_file.read_messages(topics=topics):

        if topic == '/mavros/battery':
            all_batt_messages.append(message)

    all_energy = get_amp_hours_consumed(all_batt_messages)
    print "    Total energy: {} Ah".format(all_energy)
    print "    Batt percent: {}%".format((all_energy/18.0)*100.0)
    print "    Start voltage: {}, end voltage: {}".format(all_batt_messages[0].voltage, all_batt_messages[-1].voltage)

    manip_segmented_data = get_data_for_grasp_actions(bag_file_path, ['/mavros/battery', build_phase_topic])
    energy_by_phase = {}

    for i, manip in enumerate(manip_segmented_data):
        phases = get_phase_start_times(manip, 'Grasping block')
        manip_end = manip['/droplet_underwater_assembly/build_phase'][-1].header.stamp
        manip_start = manip['/droplet_underwater_assembly/build_phase'][0].header.stamp
        manip_end_seconds = (manip_end - manip_start).to_sec()

        amps_consumed_in_manip = get_amp_hours_consumed(manip['/mavros/battery'])
        print(amps_consumed_in_manip)
        percentage_of_batt = (amps_consumed_in_manip / 18.0)*100.0
        vals.append(percentage_of_batt)
        print("      Manip {}: {:.2f}%".format(i, percentage_of_batt))

        for i, (name, start_time) in enumerate(phases):
            batt_messages_for_phase = []
            if i < len(phases)-1:
                end_time = phases[i+1][1]
            else:
                end_time = manip_end_seconds

            for m in manip['/mavros/battery']:
                m_time_seconds = (m.header.stamp - manip_start).to_sec()
                if start_time <= m_time_seconds <= end_time:
                    batt_messages_for_phase.append(m)

            if name not in energy_by_phase:
                energy_by_phase[name] = 0.0

            energy_by_phase[name] = energy_by_phase[name] + get_amp_hours_consumed(batt_messages_for_phase)

    total_in_manips = 0.0
    for phase in sorted(energy_by_phase.keys()):
        print("    {}: {} | {}% of total".format(phase, energy_by_phase[phase], (energy_by_phase[phase] / all_energy) * 100.0))
        total_in_manips = total_in_manips + energy_by_phase[phase]

    print("    {}: {} | {}% of total".format("Moving to next grasp", all_energy - total_in_manips, ((all_energy - total_in_manips) / all_energy) * 100.0))

def make_pre_drop_position_error_plots(data, release_type):
    position_errors = get_position_errors_during_drops(data, release_type)

    pyplot.scatter(
        [p[0] * 100.0 for p in position_errors],
        [p[1] * 100.0 for p in position_errors],
        c='r',
        label='Successful drop'
    )
    pyplot.scatter(
        [0.0],
        [0.0],
        c='k',
        label='Ideal drop location'
    )
    pyplot.xlabel("X error (cm)")
    pyplot.ylabel("Y error (cm)")
    pyplot.show()


def get_average_phase_time():
    pass

def get_position_errors_during_grasps():
    pass


#r = get_data_for_grasp_actions(selected_file, ['/mavros/battery', build_phase_topic])
#print(r[-1]['/droplet_underwater_assembly/build_phase'][-1])

for f in all_files:
    print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    print f
    #r = get_data_for_grasp_actions(f, ['/mavros/battery', build_phase_topic])
    #get_phase_timing_stats(r)
    get_energy_stats_by_phase(f)
    print "............................"

print("Avg percent consumed: {}".format(avg(vals)))
#r = get_data_for_grasp_actions(all_files[0], ['/mavros/battery', build_phase_topic])
#make_energy_timeline_plot(r)

#make_energy_timeline_plot(r)
#make_grasp_alignment_plot(r, 4)

#make_pre_drop_position_error_plots(r, 'open_gripper')
#errs = get_position_errors_during_drops(r, 'open_gripper')
#print(errs)

#data = [get_amp_hours_consumed(x['/mavros/battery']) * 10.0 for x in r]
#
#print "Average battery percentage consumed to carry an object: {}".format(avg(data))
#
#
#print(errs)
