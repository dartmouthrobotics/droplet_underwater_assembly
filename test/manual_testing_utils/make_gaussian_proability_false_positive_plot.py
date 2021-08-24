# 3.3 inch square.
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
import numpy as np
import math
import scipy.stats
import matplotlib.ticker as mtick

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans",
    "font.sans-serif": ["Bitstream Vera"]})


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

    return test


#x_samples = np.linspace(0.000001, 0.012, 100)
ax.legend()

cov_from_experiment = np.array([[4.32117434e-07,9.90958912e-07], [9.90958912e-07,2.71886186e-05]])

r_real = 0.042

num_r_samples = 250
r_samples = np.linspace(0.01, 0.045, num_r_samples)
worsts = []

def get_worst_case_prob(r):
    worst = float("-inf")
    worst_angle = 0.0
    angles = np.linspace(0.0, math.pi, 75)
    for angle in angles:
        x = math.cos(angle) * r
        y = math.sin(angle) * r

        p = get_prob_at_point(x=x,y=y,std_x=0,std_y=0.0,secs=10.0,framerate=30.0,cov=cov_from_experiment)

        if p > worst:
            worst_angle = angle
            worst = p

    print("  p: {} at {}".format(worst, worst_angle))
    return worst

for idx, r in enumerate(r_samples):
    print("r = {} ({}/{})".format(r, idx+1, num_r_samples))
    worst = get_worst_case_prob(r)

    worsts.append(worst)

#plt.plot(r_samples, worsts)
#plt.axvline(0.042, c="r")
#
#y_samples = []
#for idx, x_sample in enumerate(x_samples):
#    sys.stdout.write("{}/{}\r".format(idx,len(x_samples)))
#    sys.stdout.flush()
#
#    y_samples.append(
#        get_prob_at_point(0.042, 0.0, x_sample, std_y,10.0,30.0)
#    )
#
#line_min = 0.0
#for x_sam, y_sam in zip(x_samples, y_samples):
#    if y_sam > 0.01:
#        line_min = x_sam
#        break
def non_increasing(L):
    return all(x>=y for x, y in zip(L, L[1:]))    

print("Worst samples nonincreasing? {}".format(non_increasing(worsts)))
ax.set_xlabel("Meters")
ax.set_xlabel("Meters")
axs[1].plot(r_samples, worsts, label="$P(\\mathcal{{E}} | \Sigma)$")
#axs[1].xaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))
#axs[1].xaxis.set_major_locator(mtick.MultipleLocator(0.005))
axs[1].axvline(r_real, color="r", linestyle="dashed", label="$r=0.042$ $P(\\mathcal{{E}} | \Sigma)={:.0e}$".format(get_worst_case_prob(r_real)))
axs[1].set_xlabel("r (meters)")
axs[1].set_title("\\textbf{(b)}")
ax.set_title("\\textbf{(a)}")
axs[1].legend(loc="upper right")
fig.tight_layout()

fig.savefig("acceptance_area_probability_false_positive_plot.pdf")

plt.show()
