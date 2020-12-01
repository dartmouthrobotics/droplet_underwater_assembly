from matplotlib import pyplot as plt

f = open("drop_locs.csv")

fig, ax = plt.subplots()

lines = f.readlines()

x = []
y = []

x_f = []
y_f = []

goal = [-1.608,-0.115,-0.23]

yaw = []
yaw_colors = []

for line in lines:
    line_vals = line.split(",")

    if line_vals[0] == "n":
        x_f.append(float(line_vals[1]))
        y_f.append(float(line_vals[2]))
        yaw_colors.append('red')
        print line_vals
    else:
        x.append(float(line_vals[1]))
        y.append(float(line_vals[2]))
        yaw_colors.append('silver')

    yaw.append(float(line_vals[4]))

ax.set_title("Drop locations")
ax.scatter(x, y, label="success", color="silver")
ax.scatter(x_f, y_f, label="fail", color=['red'])

ax.scatter([goal[0]], [goal[1]], label="goal", color="lime", s=60)

ax.set_xlabel("Robot X (meters)")
ax.set_ylabel("Robot Y (meters)")
ax.legend()

plt.show()

fig, ax = plt.subplots()

ax.title("Yaw values")
ax.bar(list(range(len(yaw))), yaw, color=yaw_colors)

plt.show()
