from pclpy import pcl


def view_multiple(clouds):
    viewer = pcl.visualization.PCLVisualizer("viewer")

    viewer.initCameraParameters()

    n_clouds = len(clouds)
    vp_width = 1 / n_clouds
    for n, pc in enumerate(clouds, 1):
        viewer.createViewPort((n-1) * vp_width, 0.0, n * vp_width, 1.0, n)
        viewer.setBackgroundColor(0.05, 0.35, 0.6, n)
        rgb = pcl.visualization.PointCloudColorHandlerRGBAField.PointXYZRGBA(pc)
        name = "cloud%s" % n
        viewer.addPointCloud(pc, rgb, name, viewport=n)
        viewer.setPointCloudRenderingProperties(0, 2, name)

    viewer.addCoordinateSystem(1.0)
    viewer.resetCamera()

    while not viewer.wasStopped():
        viewer.spinOnce(10)
