# 3.3 inch square.
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
import numpy as np
import math
import scipy.stats
import matplotlib.ticker as mtick



half_side_length = (3.3 * 0.0254) / 2.0

tol_region_diameter = 0.08

cone_diameter = (3.2 * 0.0254)

block_cone_centers = [
    (-half_side_length, -half_side_length),
    (half_side_length, -half_side_length),
    (-half_side_length, half_side_length),
    (half_side_length, half_side_length),
    (half_side_length, 3.0*half_side_length),
    (half_side_length, -3.0*half_side_length),
    (-half_side_length, 3.0*half_side_length),
    (-half_side_length, -3.0*half_side_length),
]

fig, axs = plt.subplots(ncols=2) 
ax = axs[0]

for cone in block_cone_centers:
    c = patches.Circle(
        xy=cone,
        radius=cone_diameter / 2.0,
        zorder=2,
        fill=False,
        ec="k",
        linewidth=1.11,
        label="Receiver Cones"
    )

    ax.add_patch(c)

    c = patches.Circle(
        xy=cone,
        radius=0.1 * 0.0254,
        zorder=2,
        color="k",
    )
    ax.add_patch(c)

acceptance_area = patches.Circle(
    xy=(0,0),
    radius=(tol_region_diameter/2.0),
    zorder=3,
    facecolor='cyan',
    label="Acceptance Area"
)
ax.add_patch(acceptance_area)

tol = patches.Rectangle(xy=(-0.012, -0.012), width=0.024, height=0.024, facecolor='silver', zorder=4, label="Tolerance")
ax.add_patch(tol)

ax.set_xlim(-0.12,0.12)
ax.set_ylim(-0.22,0.22)
ax.set_aspect('equal')
ax.scatter(0, 0, c="green", label="Ideal drop location", zorder=5)

std_x = 0.000393755628946
std_y = 0.00333229802625


def get_integrand(mean_x, mean_y, cov):
    return lambda y,x: scipy.stats.multivariate_normal.pdf([x,y], mean=[mean_x,mean_y], cov=cov)


def integrand(x0, x1, mean, cov):
    return scipy.stats.multivariate_normal.pdf([x0, x1], mean=mean, cov=cov)


def get_prob_at_point(x,y,std_x,std_y,secs,framerate,cov=None):
    cov_matrix = np.array([[std_x*std_x,  0.0], [0.0,   std_y*std_y]])
    if cov is not None:
        cov_matrix = cov
    mean = [x,y]
    test, l = scipy.integrate.nquad(integrand, ranges=[[-0.012, 0.012], [-0.012, 0.012]], args=(mean, cov_matrix))
    return min(math.pow(framerate * secs * test, 2), 1.0)


x_samples = np.linspace(0.000001, 0.012, 100)
ax.legend()

cov_from_experiment = np.array([[1.55111170e-07,-3.19329906e-07], [ -3.19329906e-07,1.11090570e-05]])

angles = np.linspace(0.0, 2*math.pi, 100)
worst = float("-inf")
for angle in angles:
    r = 0.042
    x = math.cos(angle) * r
    y = math.sin(angle) * r

    p = get_prob_at_point(x=x,y=y,std_x=0,std_y=0.0,secs=10.0,framerate=30.0,cov=cov_from_experiment)
    print(x, y, p)

    if p > worst:
        worst = p

print("Worst", p)
    #print(get_prob_at_point(0.000,0.042,0.0,0.0,10.0,30.0,cov_from_experiment))

y_samples = []
for idx, x_sample in enumerate(x_samples):
    sys.stdout.write("{}/{}\r".format(idx,len(x_samples)))
    sys.stdout.flush()

    y_samples.append(
        get_prob_at_point(0.042, 0.0, x_sample, std_y,10.0,30.0)
    )

line_min = 0.0
for x_sam, y_sam in zip(x_samples, y_samples):
    if y_sam > 0.01:
        line_min = x_sam
        break
    

ax.set_xlabel("meters")
ax.set_xlabel("meters")
axs[1].plot(x_samples, y_samples, label="False positive prob.")
axs[1].xaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
axs[1].xaxis.set_major_locator(mtick.MultipleLocator(0.005))
axs[1].axvline(line_min, c="red", linestyle="dashed", label="P > 1%")
axs[1].set_xlabel("X Standard Deviation")
axs[1].set_title("(b)")
ax.set_title("(a)")
axs[1].legend()
fig.tight_layout()

fig.savefig("acceptance_area_probability_false_positive_plot.pdf")

plt.show()
