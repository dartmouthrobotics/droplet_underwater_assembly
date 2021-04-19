import rosbag
import collections
import rospy
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math

def avg(vals):
    return sum(vals) / float(len(vals))

bag_file_path = "/media/sam/storage/uw-assembly-experiments/bag_files/rss2021_fixed_robot_tests/localization_isolation_experiment_pretty_good.bag"

coordinate_time_slices = {
    (0,0):(1614091559.766033408,1614091585.891227869),
    (0,1):(1614091684.344000000,1614091742.667000000),
    (0,2):(1614091808.576000000,1614091885.642000000),
    (0,3):(1614091939.353000000,1614091982.394000000),
    (0,4):(1614092020.174000000,1614092122.844000000),
    (0,5):(1614092149.534000000,1614092183.131000000),
    (0,6):(1614092218.696000000,1614092253.688000000),
    (1,0):(1614092617.141000000,1614092654.296000000),
    (1,1):(1614092683.316000000,1614092701.337000000),
    (1,2):(1614092735.175000000,1614092762.496000000),
    (1,3):(1614092792.007000000,1614092813.431000000),
    (1,4):(1614092842.389000000,1614092871.898000000),
    (1,5):(1614092895.521000000,1614092944.005000000),
    (1,6):(1614092976.145000000,1614093013.335000000),
    (2,0):(1614093331.562863256,1614093355.704000000),
    (2,1):(1614093382.995596869,1614093406.809000000),
    (2,2):(1614093460.456000000,1614093481.214000000),
    (2,3):(1614093512.603000000,1614093534.838000000),
    (2,4):(1614093562.545396687,1614093593.819000000),
    (2,5):(1614093623.197883081,1614093641.862000000),
    (2,6):(1614093668.912000000,1614093695.946000000),
    (3,0):(1614093828.168000000,1614093843.216000000),
    (3,1):(1614093875.677617895,1614093906.015000000),
    (3,2):(1614093936.594000000,1614093960.324000000),
    (3,3):(1614093994.626149798,1614094032.843000000),
    (3,4):(1614094189.681253853,1614094199.710000000),
    (3,5):(1614094222.518000000,1614094237.446000000),
    (3,6):(1614094261.350000000,1614094281.546000000),
    (4,0):(1614094479.022000000,1614094502.620000000),
    (4,1):(1614094530.560105104,1614094551.454000000),
    (4,2):(1614094587.921883250,1614094597.127204890),
    (4,3):(1614094633.664000000,1614094652.339000000),
    (4,4):(1614094677.224818707,1614094691.793000000),
    (4,5):(1614094712.721430681,1614094730.632000000),
    (4,6):(1614094752.294097976,1614094785.180488365),
    (5,0):(1614095035.467270040,1614095048.563324799),
    (5,1):(1614095066.406018014,1614095076.065000000),
    (5,2):(1614095093.703997381,1614095107.243000000),
    (5,3):(1614095130.396922583,1614095154.160000000),
    (5,4):(1614095169.379552243,1614095186.190000000),
    (5,5):(1614095216.530000000,1614095230.711000000),
    (5,6):(1614095255.115264151,1614095269.488000000),
    (6,0):(1614095409.723000000,1614095428.533000000),
    (6,1):(1614095449.444930878,1614095472.124000000),
    (6,2):(1614095509.297469306,1614095531.120000000),
    (6,3):(1614095553.285722723,1614095565.282000000),
    (6,4):(1614095590.274707509,1614095598.577000000),
    (6,5):(1614095624.338876213,1614095638.260000000),
    (6,6):(1614095656.144696646,1614095671.574000000),
}

bag = rosbag.Bag(bag_file_path)

messages_by_loc = collections.defaultdict(list)

fig, ax = plt.subplots(figsize=(6.5,6.5))
ax.set_aspect('equal')

for topic, message, t in bag.read_messages(topics=["/droplet_underwater_assembly/build_phase"]):
    count = 0
    for key, value in coordinate_time_slices.items():
        t0 = rospy.Time.from_sec(value[0])
        t1 = rospy.Time.from_sec(value[1])
        if t0 <= t and t1 >= t:
            count = count + 1
            messages_by_loc[key].append(message)
    if count > 1:
        print("BAD BAD BAD")
        raise Exception("BAD DATA BRO")

tol_region_side_length = 0.015

slot_center = [
    avg([msg.current_location[0] for msg in messages_by_loc[(3,3)]]),
    avg([msg.current_location[1] for msg in messages_by_loc[(3,3)]]),
]

keys_with_positives = set()
for key, messages in messages_by_loc.items():
    for m in messages:
        tolx = 0.012
        toly = 0.012
        if math.fabs(slot_center[0] - m.current_location[0]) <= tolx and math.fabs(slot_center[1] - m.current_location[1]) <= toly:
            keys_with_positives.add(key)

print("Keys with positives: ")
print keys_with_positives

tol_region = patches.Rectangle(
    xy=(slot_center[0] - tol_region_side_length / 2.0, slot_center[1] - tol_region_side_length / 2.0),
    height=tol_region_side_length,
    width=tol_region_side_length,
    facecolor=(1.0,0.0,0.0),
    zorder=1,
    label="Waypoint tolerance region"
)
ax.add_patch(tol_region)

accept_region_side_length = 3.0 * 0.0254

#accept_region = patches.Rectangle(
#    xy=(slot_center[0] - accept_region_side_length / 2.0, slot_center[1] - accept_region_side_length / 2.0),
#    height=accept_region_side_length,
#    width=accept_region_side_length,
#    facecolor=(0.0,1.0,1.0),
#    zorder=0,
#    label="Acceptance Region"
#)
accept_region = patches.Ellipse(
    xy=(slot_center[0], slot_center[1]),
    height=0.080,
    width=0.080,
    facecolor=(0.0,1.0,1.0),
    zorder=0,
    label="Acceptance Region"
)
ax.add_patch(accept_region)

# does this have any false positive readings?
position_cloud_points = [[],[]]
position_cloud_centers = []

keys_with_positives = set()

for key, messages in messages_by_loc.items():
    position_cloud_points[0].extend([
        m.current_location[0] for m in messages
    ])
    position_cloud_points[1].extend([
        m.current_location[1] for m in messages
    ])


    loc_center = [
        avg([msg.current_location[0] for msg in messages]),
        avg([msg.current_location[1] for msg in messages]),
    ]

    position_cloud_centers.append(loc_center)

ax.scatter(position_cloud_points[0], position_cloud_points[1], c='silver', label="Position Readings", zorder=2)
ax.scatter(
    [c[0] for c in position_cloud_centers],
    [c[1] for c in position_cloud_centers],
    c='k',
    label='Average Location Readings',
    zorder=3
)

ax.scatter([slot_center[0]], [slot_center[1]], label="Ideal Drop Location", c='lime', zorder=4)
ax.set_xlabel("meters")
ax.set_ylabel("meters")
ax.legend()
#ax.set_title("49 locations relative to slot center")
fig.savefig("localization_isolation_results.pdf")
plt.show()
plt.cla()
plt.clf()

fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)

center_locs = messages_by_loc[(3,3)]

std_x = np.std([m.current_location[0] for m in center_locs])
std_y = np.std([m.current_location[1] for m in center_locs])
cov_arg = []

print("COV {}".format(np.cov(
    [[m.current_location[0] for m in center_locs],
    [m.current_location[1] for m in center_locs]],
)))
print("STD X {} STD Y {}".format(std_x, std_y))
ax1.hist([m.current_location[0] for m in center_locs])
ax2.hist([m.current_location[1] for m in center_locs])
ax1.axvline(avg([m.current_location[0] for m in center_locs]), color='k', linestyle='dashed', linewidth=1)
ax2.axvline(avg([m.current_location[1] for m in center_locs]), color='k', linestyle='dashed', linewidth=1)

#ax1.set_title("X readings")
#ax2.set_title("Y readings")
fig.savefig("X_Y_reading_distributions.pdf")

plt.show()
# what do we want to show for the histograms?
