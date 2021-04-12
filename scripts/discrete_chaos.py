#!/usr/bin/env python

import numpy as np
from itertools import count
from functools import partial
from matthew import show_pointcloud


def kaplan_yorke(alpha, initial_value, n, b=18014398241046527):
    def kaplan_yorke_map():
        a = 1
        x = initial_value
        yield x.copy()
        for _ in range(n - 1):
            a = (2 * a) % b
            x[0] = a / b
            x[1] = alpha * x[1] + np.cos(4 * np.pi * x[0])
            yield x.copy()

    return np.array([*kaplan_yorke_map()])


def henon(b, a, initial_value, n):
    def henon_map():
        x = initial_value
        yield x.copy()
        for _ in range(n - 1):
            x[0] = 1 - a * x[0] ** 2 + x[1]
            x[1] = b * x[0]
            yield x.copy()

    return np.array([*henon_map()])


if __name__ == "__main__":
    n_points_per_alpha = 1000
    worker = partial(henon,
                     a=1.4,
                     initial_value=np.array([0.5, 0.2]),
                     n=n_points_per_alpha)
    from multiprocessing import Pool

    n_alpha = 500
    alpha = np.linspace(-.7, 2.7, n_alpha)
    points = np.empty((n_alpha * n_points_per_alpha, 3))

    with Pool() as p:
        for i, alpha, traj in zip(count(), alpha, p.map(worker, alpha)):
            points[i * n_points_per_alpha:(i + 1) * n_points_per_alpha, 0:2] = traj
            points[i * n_points_per_alpha:(i + 1) * n_points_per_alpha, 2] = alpha

    from matplotlib.cm import get_cmap

    for i in [0, 1, 2]:
        p = points[:, i]
        points[:, i] = (p - p.min()) / (p.max() - p.min())

    show_pointcloud(points, colors=get_cmap()(points[:, 2]))

