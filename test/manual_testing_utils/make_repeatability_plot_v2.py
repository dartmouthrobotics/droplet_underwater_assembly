import rosbag
import itertools
import matplotlib.pyplot
import matplotlib.patches

bag_files = [
    "/media/sam/storage/uw-assembly-experiments/bag_files/rss2021_repeatability_tests/repeatability_50_blocks_single_start.bag",
    "/media/sam/storage/uw-assembly-experiments/bag_files/rss2021_repeatability_tests/repeatability_random_starts_1_through_43.bag",
    "/media/sam/storage/uw-assembly-experiments/bag_files/rss2021_repeatability_tests/repeatability_random_starts_44_through_50.bag",
]

drop_messages = []

for repeatability_bag_file_path in bag_files: 
    print("Reading bag file {}".format(repeatability_bag_file_path))
    bag = rosbag.Bag(repeatability_bag_file_path)

    num_messages = 0

    for topic, msg, t in bag.read_messages(topics=['/droplet_underwater_assembly/build_phase']):
        if msg.current_action_type == 'open_gripper' and msg.current_action_is_started:
            drop_messages.append(msg)

first_drops = []
for (key, messages) in itertools.groupby(drop_messages, key=lambda x: x.action_sequence_id):
    messages = list(messages)
    message = messages[0]

    x_err = abs(message.goal_location[0] - message.current_location[0])
    y_err = abs(message.goal_location[1] - message.current_location[1])
    x_tol = message.move_tolerance[0]
    y_tol = message.move_tolerance[1]

    if x_err <= x_tol and y_err <= y_tol:
        first_drops.append(message)
    else:
        print("BAAAAA")

print(len(first_drops))

times = []
tot = 0.0
num_bags = 0

x_pos = []
y_pos = []

goals = set()
tols = set()

for idx in range(1, len(first_drops)): 
    if first_drops[idx].action_sequence_id < first_drops[idx-1].action_sequence_id:
        num_bags = num_bags + 1
        continue

    if idx == 1:
        x_posn = first_drops[idx-1].current_location[0]
        y_posn = first_drops[idx-1].current_location[1]
        x_pos.append(x_posn)
        y_pos.append(y_posn)

    x_posn = first_drops[idx-1].current_location[0]
    y_posn = first_drops[idx-1].current_location[1]

    if num_bags > 0:
        x_posn = x_posn + 0.002

    x_pos.append(x_posn)
    y_pos.append(y_posn)

    x_err = abs(first_drops[idx].goal_location[0] - first_drops[idx].current_location[0])
    y_err = abs(first_drops[idx].goal_location[1] - first_drops[idx].current_location[1])
    x_tol = first_drops[idx].move_tolerance[0]
    y_tol = first_drops[idx].move_tolerance[1]

    goals.add(first_drops[idx].goal_location)
    tols.add(first_drops[idx].move_tolerance)

    if x_err <= x_tol and y_err <= y_tol:
        tot = tot + 1.0
        times.append(
            (first_drops[idx].header.stamp - first_drops[idx-1].header.stamp).to_sec()
        )
    else:
        print("Baddddd")

print("Average time: {}".format(sum(times) / tot))
print "Goals {}".format(goals)
print "Tols {}".format(tols)

x_tol = first_drops[0].move_tolerance[0] + 0.000
y_tol = first_drops[0].move_tolerance[1] + 0.000

x_goal = [first_drops[0].goal_location[0]]
y_goal = [first_drops[0].goal_location[1]]

fig, ax = matplotlib.pyplot.subplots()
ax.set_aspect('equal')
ax.scatter(x_pos, y_pos, label="Drop location", zorder=1, c="k")
ax.scatter(x_goal, y_goal, label="Ideal drop location", zorder=2, c="orange")
#ax.set_title("100 Successful Drop Locations")
ax.set_aspect('equal')
r = matplotlib.patches.Rectangle(
    width=x_tol * 2.0,
    height=y_tol * 2.0,
    zorder=0,
    xy=(x_goal[0] - x_tol, y_goal[0] - y_tol),
    facecolor="silver",
    label="Tolerance Region"
)
ax.add_patch(r)
ax.set_xlabel("meters")
ax.set_ylabel("meters")

ax.legend()
fig.savefig("repeatability_results.pdf")

matplotlib.pyplot.show()
