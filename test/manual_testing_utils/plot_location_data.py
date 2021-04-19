"""
Plots position data from a csv file for the position/localization accuracy experiment for RSS2021
"""

from matplotlib import pyplot, mlab
import statistics
import scipy.integrate
import numpy
import math

def gaussian_prob(x, stddev, mean):
    l = 1.0 / (math.sqrt(2.0 * math.pi * stddev**2))
    r = math.pow((x - mean) / stddev, 2)

    return l * math.exp(-0.5 * r)

# dimensions of square: 3.3 inches = 0.08382 

square_side = 0.08382 
half_square_side = square_side / 2.0

tol_side = 0.020

data = [x.strip().split(",") for x in open("output.csv").readlines()]

print(data[0][-1] == "0\n")
x_pos = [float(l[0]) for l in data if l[-1] == "0"]
y_pos = [float(l[1]) for l in data if l[-1] == "0"]

x_stddev = statistics.stdev(x_pos)
y_stddev = statistics.stdev(y_pos)
print("XY stddev")
print(x_stddev)
print(y_stddev)
print("AAA")

n, bins, patches = pyplot.hist(y_pos, bins=15)
pyplot.axvline(statistics.mean(y_pos), color='k', linestyle='dashed', linewidth=1)
y = mlab.normpdf(bins, statistics.mean(y_pos), y_stddev)
pyplot.plot(bins, y, 'r--')

pyplot.show()

print("x_range: {} to {}. Length: {}".format(min(x_pos), max(x_pos), max(x_pos) - min(x_pos)))
print("y_range: {} to {}. Length: {}".format(min(y_pos), max(y_pos), max(y_pos) - min(y_pos)))

avg_pt = (statistics.mean(y_pos), statistics.mean(x_pos))

pyplot.scatter(y_pos, x_pos)
pyplot.scatter(*avg_pt)

square_corners = [
    (avg_pt[0] + half_square_side, avg_pt[1] + half_square_side),
    (avg_pt[0] + half_square_side, avg_pt[1] - half_square_side),
    (avg_pt[0] - half_square_side, avg_pt[1] - half_square_side),
    (avg_pt[0] - half_square_side, avg_pt[1] + half_square_side),
]

pyplot.scatter([c[0] for c in square_corners], [c[1] for c in square_corners])

square_corners = [
    (avg_pt[0] + tol_side, avg_pt[1] + tol_side),
    (avg_pt[0] + tol_side, avg_pt[1] - tol_side),
    (avg_pt[0] - tol_side, avg_pt[1] - tol_side),
    (avg_pt[0] - tol_side, avg_pt[1] + tol_side),
]

pyplot.scatter([c[0] for c in square_corners], [c[1] for c in square_corners])

# one cm outside the bounds
y_test = avg_pt[0] + half_square_side + 0.005

res = scipy.integrate.quad(lambda y: gaussian_prob(y, y_stddev, y_test), avg_pt[0] - tol_side, avg_pt[0] + tol_side)
print("false positive probability on y: {}".format(
    res
))
print("Num seconds for good chance of false positive", (0.5 / res[0]) / 30.0)

x_pos = [float(l[0]) for l in data if l[-1] == "2"]
y_pos = [float(l[1]) for l in data if l[-1] == "2"]

print("x_range: {} to {}. Length: {}".format(min(x_pos), max(x_pos), max(x_pos) - min(x_pos)))
print("y_range: {} to {}. Length: {}".format(min(y_pos), max(y_pos), max(y_pos) - min(y_pos)))

avg_pt = (statistics.mean(y_pos), statistics.mean(x_pos))

pyplot.scatter(y_pos, x_pos)
pyplot.scatter(*avg_pt)

x_pos = [float(l[0]) for l in data if l[-1] == "2.5"]
y_pos = [float(l[1]) for l in data if l[-1] == "2.5"]

print("x_range: {} to {}. Length: {}".format(min(x_pos), max(x_pos), max(x_pos) - min(x_pos)))
print("y_range: {} to {}. Length: {}".format(min(y_pos), max(y_pos), max(y_pos) - min(y_pos)))

avg_pt = (statistics.mean(y_pos), statistics.mean(x_pos))

pyplot.scatter(y_pos, x_pos)
pyplot.scatter(*avg_pt)

x_pos = [float(l[0]) for l in data if l[-1] == "2.25"]
y_pos = [float(l[1]) for l in data if l[-1] == "2.25"]

print("x_range: {} to {}. Length: {}".format(min(x_pos), max(x_pos), max(x_pos) - min(x_pos)))
print("y_range: {} to {}. Length: {}".format(min(y_pos), max(y_pos), max(y_pos) - min(y_pos)))

avg_pt = (statistics.mean(y_pos), statistics.mean(x_pos))

pyplot.scatter(y_pos, x_pos)
pyplot.scatter(*avg_pt)

pyplot.show()
