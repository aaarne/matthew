#!/usr/bin/env python3


from matthew import show_pointcloud, show_mesh
import numpy as np

x = np.random.uniform(size=(10000, 3))
show_pointcloud(x)

import pymesh

bunny = pymesh.load_mesh("../data/bunny.off")
show_mesh(bunny)


