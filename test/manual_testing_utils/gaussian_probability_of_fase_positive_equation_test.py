import scipy.integrate
import math
import matplotlib.pyplot
import numpy

center_pt = [-1.632, -0.1]
tol_size = [0.015, 0.015]

x_stddev = 0.001342
y_stddev = 0.007393

number_of_seconds = 10.0
frames_per_second = 30

# multivariate gaussian distribution...
# need covariance matrix of xy values
# need vector of mean values
