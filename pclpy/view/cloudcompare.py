import subprocess
import tempfile
import os

import pclpy


def cloudcompare(data):
    # todo, check for base pointcloud type
    if not isinstance(data, (tuple, list, str)):
        temp = tempfile.NamedTemporaryFile(suffix=".las").name
        pclpy.io.to_las(data, temp)
        data = temp

    if not isinstance(data, (tuple, list)):
        data = [data]

    commands = ["CloudCompare.exe", *data]

    subprocess.Popen(commands)
