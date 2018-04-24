import pytest
import numpy as np

import pclpy


def test_eigen_vectorxf():
    a = np.array([1, 1, 1, 1], "f")
    vec = pclpy.pcl.vectors.VectorXf(a)
    assert np.allclose(np.array(vec), a)
