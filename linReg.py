# Scientific libraries
from numpy import arange, array, ones
from scipy import stats
import numpy as np


def linearRegression(data):
    xi = []
    y = []
    weights = []
    start = 0
    for (_, _, z, _, _) in data:
        y.append(z)
        xi.append(start)
        if start < len(data) / 4:
            weights.append(3)
        elif start < len(data) / 2:
            weights.append(2)
        else:
            weights.append(1)
        start = start + 1

    A = array([xi, ones(start)])

    # calculate polynomial
    z1 = np.polyfit(xi, y, 1, None, False, weights, False)
    f1 = np.poly1d(z1)

    # calculate new x's and y's
    x_new1 = xi
    y_new1 = f1(x_new1)

    return y_new1