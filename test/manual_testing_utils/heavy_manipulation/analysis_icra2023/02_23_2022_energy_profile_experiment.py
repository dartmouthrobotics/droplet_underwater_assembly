import rospy
import rosbag
import numpy
import itertools
from matplotlib import pyplot

pyplot.rcParams.update({
  "text.usetex": True,
  #"font.family": "Helvetica"
})

# for pos
pos_bag_file_path = "/media/sam/storage/uw-assembly-experiments/heavy_manipulation/buoyancy-calibration-experiment-feb-28-2022/calib_pos_buoyancy_2021-07-28-15-13-56.bag"

# for neg
neg_bag_file_path = "/media/sam/storage/uw-assembly-experiments/heavy_manipulation/negative_ballast_calibration_05_04/ballast_calib_neg_changes_2021-07-28-15-20-14.bag"


neg_bag_file = rosbag.Bag(neg_bag_file_path)
pos_bag_file = rosbag.Bag(pos_bag_file_path)


def avg(stuff):
    stuff2 = itertools.tee(stuff)

    return sum(stuff) / float(len(stuff))

def plot_ballast_data(positive, ax, bag_file):
    hold_str = "HOLD 30.0"

    if positive:
        hold_str = "HOLD 35.0"

    action_str = None
    action_seq = None
    hold_number = 0
    hold_costs = []
    energy_usages = []
    voltages = []

    for topic, message, time in bag_file.read_messages(topics=["/mavros/battery", "/droplet_underwater_assembly/build_phase"]):
        if topic == "/mavros/battery":
            voltages.append(message.voltage)
            if action_str is not None:
                if action_str == hold_str: # 30.0 for neg
                    energy_usages.append(
                        (-1 * message.current, action_seq)
                    )

        else:
            action_str = message.active_build_step
            action_seq = message.action_sequence_id

    if len(energy_usages) == 0:
        print "Error!"
        print positive
        raise Exception()
    avgs = []
    for key, group in itertools.groupby(energy_usages, lambda x: x[1]):
        avgs.append(avg([x[0] for x in group]))

    most = max(avgs)
    ballast_vals = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

    if not positive:
        ballast_vals.reverse()

    p = numpy.polyfit(ballast_vals, avgs, 2)

    ax.plot(ballast_vals, [p[0] * b ** 2 + p[1] * b + p[2] for b in ballast_vals], label="Approximated $f(b)$")
    ax.plot(ballast_vals, avgs, label="Measured")

    if positive:
        ax.set_title("With block")
        ax.legend()
    else:
        ax.set_title("Without block")
        ax.set_ylabel("Amps")

    ax.set_xlabel("ballast level: $b$")

fig, [ax1, ax2] = pyplot.subplots(1, 2)
fig.suptitle("Average cost to maintain depth")
plot_ballast_data(False, ax1, neg_bag_file)
plot_ballast_data(True, ax2, pos_bag_file)

pyplot.show()
