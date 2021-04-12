#!/usr/bin/env python

import numpy as np
from matthew import show_pointcloud


def lorentz(x, rho, sigma, beta):
    return np.array([
        sigma * (x[1] - x[0]),
        x[0] * (rho - x[2]) - x[1],
        x[0] * x[1] - beta * x[2]
    ])


if __name__ == "__main__":
    from scipy.integrate import solve_ivp as ode45
    sol = ode45(
        fun=lambda t, y: lorentz(y, rho=28, sigma=10, beta=2.666),
        t_span=[0, 100.0],
        y0=np.array([1, 0, 0]),
        method='RK45',
        max_step=1e-3,
        dense_output=True)

    points = sol.y.T

    print(f"Solver finished with success={sol.success}. Trajectory has {points.shape[0]} points.")
    from matplotlib.cm import get_cmap
    from matplotlib.colors import Normalize
    time_normalizer = Normalize(vmin=np.min(sol.t), vmax=np.max(sol.t))
    cm = get_cmap()
    colors = cm(time_normalizer(sol.t))
    print(f"Using {cm.name} colormap to encode time.")

    show_pointcloud(points, colors=colors)

