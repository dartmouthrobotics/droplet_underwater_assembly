import rosbag
import rospy
import itertools
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import collections
from matplotlib.ticker import FormatStrFormatter

# what are the phases we want to show?
#    the robot repositions, grabs a block, repositions, drops the block
#    what would be interesting to show though?
#    The phases could be the different tolerances we use or they could be the action types
#    so that could be like "move to new platform", "reposition to block", "grasp block", "reposition to back point", "move to new platform", ....

ActionChunk = collections.namedtuple("ActionChunk", ['start_time', 'end_time', 'action'])

bag_file_path = "/media/sam/storage/uw-assembly-experiments/bag_files/seven_block_build_successful_with_good_video.bag"

bag = rosbag.Bag(bag_file_path)

action_chunks = []

first_time = None

for group, messages in itertools.groupby(bag.read_messages(topics=['/droplet_underwater_assembly/build_phase']), key=lambda data: data[1].current_action_type):
    print "Reading messages. At action " + group
    messages_list = list(messages)
    start_time = messages_list[0][1].header.stamp
    end_time = messages_list[-1][1].header.stamp

    if first_time is None:
        first_time = start_time 

    action_chunks.append(
        ActionChunk(
            start_time=start_time,
            end_time=end_time,
            action=group
        )
    )

time_by_action = collections.defaultdict(lambda: 0.0)

for chunk in action_chunks:
    time_by_action[chunk.action] = time_by_action[chunk.action] + (chunk.end_time - chunk.start_time).to_sec()

print time_by_action

fig, ax = plt.subplots(figsize=(5,2.5))

y_val = 0.0
height = 1.0

ax.set_xlim(
    0.0,
    (action_chunks[-1].end_time - first_time).to_sec()
)

y_padding = 0.15

ax.set_ylim(
    y_val,
    y_val + height + y_padding
)

total_time_by_type = collections.defaultdict(lambda: 0.0)

name_remapping = {
    "move": "Reposition",
    "binary_P_move": "Switch Platforms",
    "change_platforms": "Switch Platforms",
    "move_wrist": "Rotate Gripper",
    "close_gripper": "Close Gripper",
    "open_gripper": "Waiting to drop"
}

color_by_type = {
    "Reposition": (200.0/255.0, 200.0/255.0, 200.0/255.0),
    "Switch Platforms": (0.0, 1.0, 1.0),
    "Waiting to drop": (1.0, 0.0, 0.5),
    "Close Gripper": (255.0/255.0, 131.0/255.0, 0.0/255.0),
    "Rotate Gripper": (1.0, 1.0, 0.0)
}

for chunk in action_chunks:
    name = name_remapping[chunk.action]
    rect = patches.Rectangle(
        xy=((chunk.start_time - first_time).to_sec(), y_val),
        height=height,
        width = (chunk.end_time - chunk.start_time).to_sec(),
        facecolor=color_by_type[name],
        label=name
    )
    total_time_by_type[chunk.action] = total_time_by_type[chunk.action] + (chunk.end_time - chunk.start_time).to_sec()
    ax.add_patch(rect)

ax.get_yaxis().set_visible(False)
ax.legend(loc="upper left")
ax.spines['top'].set_visible(False)
ax.spines['left'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.set_xticks([0, 700, 1400])
ax.xaxis.set_major_formatter(FormatStrFormatter('%d(s)'))
fig.savefig("seven_block_build_timeline.pdf")
plt.show()
