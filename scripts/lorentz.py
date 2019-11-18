#!/usr/bin/env python

import numpy as np


def lorentz(x, rho, sigma, beta):
    return np.array([
        sigma * (x[1] - x[0]),
        x[0] * (rho - x[2]) - x[1],
        x[0] * x[1] - beta * x[2]
    ])


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


if __name__ == "__main__":
    from scipy.integrate import solve_ivp as ode45
    sol = ode45(
        fun=lambda t, y: lorentz(y, 28, 10, 8.0 / 3),
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

    exporter = PCDExporter(points)
    exporter.add_color(colors)
    pcdfile = "/tmp/lorentz.pcd"
    with open(pcdfile, "w") as f:
        exporter.write_to(f)
        print(f"Point cloud written to {pcdfile}.")

    import subprocess
    subprocess.run(['../build/matthew/matthew', '--file', pcdfile, '--background', "0", "0", "0"])

