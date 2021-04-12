import subprocess
import numpy as np
import tempfile
from .pcdexporter import PCDExporter


MATTHEW_PATH = 'matthew'


def _base_command(fullscreen=False, additional_data_folder=None, background_color=None):
    command = [MATTHEW_PATH]
    if fullscreen:
        command.append('--fs')
    if additional_data_folder:
        command.extend(('-a', additional_data_folder))
    if background_color:
        command.append('--background')
        for c in background_color:
            command.append(str(c))
    return command


def show_mesh(mesh, asynchronous=False, **kwargs):
    import pymesh
    with tempfile.NamedTemporaryFile(mode='w', suffix='.msh', delete=not asynchronous) as f:
        pymesh.save_mesh(f.name, mesh, *mesh.get_attribute_names())
        (subprocess.Popen if asynchronous else subprocess.run)([*_base_command(**kwargs), '--file', f.name])


def show_pointcloud(points, colors=None, asynchronous=False, **kwargs):
    exporter = PCDExporter(points)
    if colors is not None:
        exporter.add_color(colors)
    if 'background_color' not in kwargs:
        kwargs['background_color'] = (0, 0, 0)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.pcd', delete=not asynchronous) as f:
        exporter.write_to(f)
        f.flush()

        command = [*_base_command(**kwargs), '--file', f.name]
        if asynchronous:
            subprocess.Popen(command)
        else:
            subprocess.run(command)
