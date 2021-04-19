import rospy
import numpy
import rosbag
import itertools
from matplotlib import pyplot
import matplotlib.colors

def avg(vals):
    return float(sum(vals)) / float(len(vals))

print "Reading bag file..."
#bag_file = rosbag.Bag('/media/sam/storage/uw-assembly-experiments/bag_files/rss2021_robot_tolerance_test/_2021-02-15-10-00-44.bag')
bag_file = rosbag.Bag('/media/sam/storage/uw-assembly-experiments/bag_files/rss2021_robot_tolerance_test/tolerance_test_three_point_triangle_grid_search.bag')

print "Done!"
messages = []
for topic, message, time in bag_file.read_messages(topics="/droplet_underwater_assembly/build_phase"):
    if message.action_sequence_id > 0:
        messages.append(message)

time_averaged = []
for tol, messages_for_tol in itertools.groupby(messages, key=lambda x: x.move_tolerance):
    first_message = next(messages_for_tol)
    last = None
    for last in messages_for_tol:
        continue

    time_at_tol = (last.header.stamp - first_message.header.stamp).to_sec()
    time_averaged.append(([tol[0], tol[1]], time_at_tol))

time_in_rows = []

x_vals = []
y_vals = []

for x_tol, data in itertools.groupby(time_averaged, key=lambda v: v[0][0]):
    data = list(data)

    x_vals.append(x_tol * 100.0)
    y_vals = [tol[0][1] * 100.0 for tol in data]

    time_in_rows.append(
        [x[1] for x in data]
    )

x_vals.reverse()
time_in_rows.reverse()
time_in_rows = numpy.array(time_in_rows).transpose()

fig, ax = pyplot.subplots()
fig.gca().invert_yaxis()
#ax.set_title("Time to traverse triangle")
ax.set_aspect('equal')

cmap = matplotlib.colors.ListedColormap(['white', 'cyan', 'blue', (float(125) / float(255), 0.0, 1.0), 'magenta', 'red'])
im = ax.imshow(time_in_rows, cmap=cmap)
ax.set_aspect('equal')
ax.set_xlabel("X tolerance (cm)")
ax.set_ylabel("Y tolerance (cm)")
ax.set_xticks(list(range(time_in_rows.shape[1])))
ax.set_yticks(list(range(time_in_rows.shape[0])))
ax.set_xticklabels(x_vals)
ax.set_yticklabels(y_vals)
ax.set_aspect('equal')

cbar = ax.figure.colorbar(im, ax=ax)
cbar.ax.set_ylabel('Seconds')

fig.savefig("tolerance_time_plot.pdf")
pyplot.show()
