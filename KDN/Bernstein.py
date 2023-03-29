import math
import numpy as np
import matplotlib.pyplot as plt

def nCr(n, r):
    """r-combination of n elements"""
    return int(math.factorial(n) / math.factorial(r) / math.factorial(n-r))

# Bernstein

def berstein(n, i, t, T):
    """Bernstein base 

    """
    return nCr(n, i) * (T - t) ** (n - i) * (t) ** (i) / (T ** n)

class BernsteinPolynomial:
    def __init__(self, control_points, T=1):
        self.control_points = control_points
        self.n = len(control_points)
        self.T = T

    def __call__(self, x):
        return sum([berstein(self.n - 1, i, x, self.T) * self.control_points[i] for i in range(self.n)])

    def plot(self, show_control_points=True, point_num=100, ax=None, x_offset=0, y_offset=0, ret_points=False, **kargs):
        t = np.linspace(0, self.T, point_num) + x_offset
        y = [self(x) + y_offset for x in np.linspace(0, self.T, point_num)]

        if ax is None:
            ax = plt.axes()

        if show_control_points:
            t2 = [self.T / (self.n - 1) * i + x_offset for i in range(self.n)]

            y2 = [y + y_offset for y in self.control_points]
            ax.plot(t2, y2, 'o--')

        ax.plot(t, y, **kargs)
        if ret_points:
            return (t,y)

        return ax

    def deriv(self):
        c = self.control_points
        c1 = [  (c[i + 1] - c[i]) * (self.n - 1) / self.T for i in range(len(c) - 1)]
        return BernsteinPolynomial(c1, T = self.T)


