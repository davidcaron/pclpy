import sys

import pclpy
from pclpy import pcl

from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setup_ui()

    def setup_ui(self):
        self.viewer = pcl.visualization.PCLVisualizer("cloud", False)

        window = self.viewer.getRenderWindow()
        self.vtk_widget = QVTKRenderWindowInteractor(self, rw=window)
        self.viewer.setupQWidget(self.vtk_widget)

        self.vtk_widget.Initialize()

        central_widget = QtWidgets.QWidget(self)
        layout = QtWidgets.QVBoxLayout(central_widget)
        layout.addWidget(self.vtk_widget)

        label = QtWidgets.QLabel("Point size: ", self)
        layout.addWidget(label)

        self.slider = QtWidgets.QSlider(central_widget)
        self.slider.setMaximumWidth(100)
        layout.addWidget(self.slider)

        self.slider.setOrientation(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(6)

        self.setCentralWidget(central_widget)

        self.slider.sliderReleased.connect(self.update_point_size)

        self.show()

    def update_point_size(self):
        self.viewer.setPointCloudRenderingProperties(pcl.visualization.PCL_VISUALIZER_POINT_SIZE,
                                                     self.slider.value(),
                                                     "cloud")
        self.vtk_widget.update()

    def add_point_cloud(self, pc):
        self.viewer.addPointCloud(pc)
        self.viewer.resetCamera()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    path = r"..\tests\test_data\street.las"
    pc = pclpy.io.read(path, "PointXYZRGBA")
    main_window.add_point_cloud(pc)
    app.exec_()
