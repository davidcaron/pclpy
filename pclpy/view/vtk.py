import numpy as np

from pclpy import pcl


class Viewer:
    BG_COLOR = (0.05, 0.35, 0.6)

    def __init__(self, *clouds, overlay=True):
        self.clouds = clouds
        self.overlay = overlay

        self.viewer = pcl.visualization.PCLVisualizer("viewer")

        self.viewer.initCameraParameters()

        if overlay:
            self.viewer.setBackgroundColor(*self.BG_COLOR, 0)
            for n, pc in enumerate(clouds, 1):
                handler = make_color_handler(pc)
                name = "cloud%s" % n
                self.viewer.addPointCloud(pc, handler, name, viewport=0)
                self.viewer.setPointCloudRenderingProperties(0, 3, name)
        else:
            n_clouds = len(clouds)
            vp_width = 1 / n_clouds
            for n, pc in enumerate(clouds, 1):
                self.viewer.createViewPort((n - 1) * vp_width, 0.0, n * vp_width, 1.0, n)
                self.viewer.setBackgroundColor(*self.BG_COLOR, n)
                handler = make_color_handler(pc)
                name = "cloud%s" % n
                self.viewer.addPointCloud(pc, handler, name, viewport=n)
                self.viewer.setPointCloudRenderingProperties(0, 3, name)

        self.viewer.addCoordinateSystem(1.0)
        self.viewer.resetCamera()

    def show(self):
        while not self.viewer.wasStopped():
            self.viewer.spinOnce(5)


def make_color_handler(pc):
    if isinstance(pc, pcl.PointCloud.PointXYZRGBA):
        return pcl.visualization.PointCloudColorHandlerRGBAField.PointXYZRGBA(pc)
    elif isinstance(pc, pcl.PointCloud.PointXYZRGB):
        return pcl.visualization.PointCloudColorHandlerRGBField.PointXYZRGB(pc)
    elif isinstance(pc, pcl.PointCloud.PointXYZ):
        return pcl.visualization.PointCloudColorHandler.PointXYZ()
    elif isinstance(pc, pcl.PointCloud.PointXYZI):
        return pcl.visualization.PointCloudColorHandlerGenericField.PointXYZI(pc, "intensity")
