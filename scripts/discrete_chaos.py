#!/usr/bin/env python

import numpy as np
from itertools import count
from functools import partial


class PCDExporter:
    def __init__(self, points):
        self._points = points
        self._color = None

    def add_color(self, color):
        assert self._points.shape[0] == color.shape[0], "There must be as many colors as points"
        c = np.array(255 * color, dtype=int)
        self._color = c[:, 0] + 256 * c[:, 1] + 256 ** 2 * c[:, 2] + 256 ** 3 * c[:, 3]

    def write_to(self, file):
        file.write(f"""# .PCD v.5 - Point Cloud Data file format
VERSION .5
FIELDS x y z{" rgba" if self._color is not None else ""}
SIZE 4 4 4{" 4" if self._color is not None else ""} 
TYPE F F F{" U" if self._color is not None else ""}
COUNT 1 1 1{" 1" if self._color is not None else ""} 
WIDTH {self._points.shape[0]}
HEIGHT 1
POINTS {self._points.shape[0]}
DATA ascii
""")
        for i, point in enumerate(self._points):
            file.write(f"{point[0]} {point[1]} {point[2]}{f' {self._color[i]}' if self._color is not None else ''}\n")


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

    exporter = PCDExporter(points)
    exporter.add_color(get_cmap()(points[:, 2]))
    pcdfile = "/tmp/discrete_chaos.pcd"
    with open(pcdfile, "w") as f:
        exporter.write_to(f)

    import subprocess
    from os.path import dirname, realpath, join

    subprocess.run([
        f'{join(dirname(realpath(__file__)), "../build/matthew/matthew")}',
        '--file', pcdfile,
        '--background', "0", "0", "0", '--fullscreen'])
