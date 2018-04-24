from pclpy import pcl


BG_COLOR = (0.05, 0.35, 0.6)

def make_color_handler(pc):
    if isinstance(pc, pcl.PointCloud.PointXYZRGBA):
        return pcl.visualization.PointCloudColorHandlerRGBAField.PointXYZRGBA(pc)
    elif isinstance(pc, pcl.PointCloud.PointXYZRGB):
        return pcl.visualization.PointCloudColorHandlerRGBField.PointXYZRGB(pc)
    elif isinstance(pc, pcl.PointCloud.PointXYZ):
        return pcl.visualization.PointCloudColorHandlerRandom.PointXYZ()


def view_multiple(*clouds, overlay=True):
    viewer = pcl.visualization.PCLVisualizer("viewer")

    viewer.initCameraParameters()

    if overlay:
        viewer.setBackgroundColor(*BG_COLOR, 0)
        for n, pc in enumerate(clouds, 1):
            handler = make_color_handler(pc)
            name = "cloud%s" % n
            viewer.addPointCloud(pc, handler, name, viewport=0)
            viewer.setPointCloudRenderingProperties(0, 3, name)
    else:
        n_clouds = len(clouds)
        vp_width = 1 / n_clouds
        for n, pc in enumerate(clouds, 1):
            viewer.createViewPort((n - 1) * vp_width, 0.0, n * vp_width, 1.0, n)
            viewer.setBackgroundColor(*BG_COLOR, n)
            handler = make_color_handler(pc)
            name = "cloud%s" % n
            viewer.addPointCloud(pc, handler, name, viewport=n)
            viewer.setPointCloudRenderingProperties(0, 3, name)

    viewer.addCoordinateSystem(1.0)
    viewer.resetCamera()

    while not viewer.wasStopped():
        viewer.spinOnce(5)
