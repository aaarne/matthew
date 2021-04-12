import numpy as np


class PCDExporter:
    def __init__(self, points):
        self._points = points
        self._color = None

    def add_color_for_all(self, color):
        c = np.empty((self._points.shape[0], 4))
        if len(color) == 4:
            c[:, :] = color
        elif len(color) == 3:
            c[:, 0:3] = color
            c[:, 3] = 1
        else:
            raise ValueError
        self.add_color(c)

        return self

    def add_color(self, color):
        assert self._points.shape[0] == color.shape[0], "There must be as many colors as points"
        c = np.array(255 * color, dtype=int)
        self._color = c[:, 0] + 256 * c[:, 1] + 256 ** 2 * c[:, 2] + 256 ** 3 * c[:, 3]

        return self

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
            file.write(
                f"{point[0]} {point[1]} {point[2]}{f' {self._color[i]}' if self._color is not None else ''}\n")

