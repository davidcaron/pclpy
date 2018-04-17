import os

import laspy
import numpy as np

import pytest

from pclpy import pcl
import pclpy


def test_data(*args):
    return os.path.join("test_data", *args)


def test_mls():
    import time
    t = time.time()
    point_cloud = pclpy.io.read_las(test_data("street.las"))
    mls = pcl.surface.MovingLeastSquaresOMP.PointXYZRGBA_PointNormal()
    mls.setSearchRadius(0.05)
    mls.setPolynomialFit(False)
    mls.setNumberOfThreads(12)
    mls.setInputCloud(point_cloud)
    tree = pcl.search.KdTree.PointXYZRGBA()
    mls.setSearchMethod(tree)
    mls.setComputeNormals(True)
    output = pcl.PointCloud.PointNormal()
    mls.process(output)
    print(dir(mls))
    print(time.time() - t)
    # pclpy.io.to_las(output, test_data("street2.las"))


def test_extract_clusters():
    print(dir(pcl.segmentation))
    input = pclpy.io.read_las(test_data("street.las"))
    import time
    t = time.time()
    ec = pcl.segmentation.EuclideanClusterExtraction.PointXYZRGBA()
    ec.input_cloud = input
    ec.min_cluster_size = 1000
    ec.max_cluster_size = 10000000
    ec.cluster_tolerance = 0.1
    indices = pcl.vector.PointIndices()
    print(len(indices))
    ec.extract(indices)
    print(len(indices))
    print(len(indices[0].indices))
    print(time.time() - t)
    # pclpy.io.to_las(output, test_data("street2.las"))


def test_compute_normals():
    input = pclpy.io.read_las(test_data("street.las"), read_colors=False)
    norm = pcl.features.NormalEstimationOMP.PointXYZ_Normal()
    norm.input_cloud = input
    norm.radius_search = 0.1
    output = pcl.PointCloud.Normal()
    norm.compute(output)
    assert np.any(output.normals)
    assert np.any(output.curvature)
    # pclpy.io.to_las(output, test_data("street2.las"))


def test_voxel_grid():
    input = pclpy.io.read_las(test_data("street.las"))
    vg = pcl.filters.VoxelGrid.PointXYZRGBA()
    vg.input_cloud = input
    vg.set_leaf_size(0.1, 0.1, 0.1)
    output = pcl.PointCloud.PointXYZRGBA()
    vg.filter(output)
    assert output.size() == 33693

# def test_diff_of_normals():
#     pc = pclpy.io.read_las(test_data("street2.las"), read_colors=True)
#     n = pcl.PointCloud.PointNormal()
#     pc.estimate_normals_omp(n, search_radius=0.25)
#     data = np.array([pc.x, pc.y, pc.z, n.normal_x, n.normal_y, n.normal_z, n.curvature]).T
#     normals_large2 = pcl.PointCloud.PointNormal.from_array(data)
#     pclpy.view.cloudcompare(normals_large2)
#     # normals_small = pcl.PointCloud.PointNormal()
#     # pc.estimate_normals_omp(normals_small, search_radius=0.15)
#     # pclpy.view.cloudcompare(normals_small)
#
#     # don = pc.difference_of_normals(normals_large, normals_small)
#     print("normals ok")
#     return
#
#     # pclpy.io.to_las(do)
#
#
#
#
#
# def test_supervoxel_clustering():
#     pc = pclpy.io.read_las(test_data("street2.las"), read_colors=True)
#     normals = pcl.PointCloud.Normal()
#     pc.estimate_normals_omp(normals, search_radius=0.10)
#     voxel_resolution = 0.03
#     seed_resolution = 0.1
#     super, clusters = pc.supervoxel_clustering(normals,
#                                                voxel_resolution=voxel_resolution,
#                                                seed_resolution=seed_resolution,
#                                                color_importance=1,
#                                                spatial_importance=5,
#                                                normal_importance=2,
#                                                use_single_camera_transform=False,
#                                                refine_supervoxels=True)
#     adjacency = super.get_supervoxel_adjacency()
#     cpc_labeled_cloud = super.get_labeled_cloud()
#     cpc = pcl.cpc_segmentation(clusters,
#                                adjacency,
#                                voxel_resolution=voxel_resolution,
#                                seed_resolution=seed_resolution,
#                                concavity_tolerance_threshold=10,
#                                use_sanity_criterion=True,
#                                max_cuts=25,
#                                cutting_min_segments=400,
#                                min_cut_score=0.2,
#                                use_local_constrain=False,
#                                use_directed_cutting=True,
#                                use_clean_cutting=True,
#                                ransac_iterations=10000,
#                                smoothness_threshold=1,
#                                min_segment_size=0,
#                                use_extended_convexity=True)
#     cpc.relabel_cloud(cpc_labeled_cloud)
#     pclpy.io.to_las(cpc_labeled_cloud, test_data("street_out.las"))
#     pclpy.view.cloudcompare(cpc_labeled_cloud)
#
#
# def test_region_growing_large():
#     f = laspy.file.File("D:\Projects\cloudcompare_fix_las_write\Track_EF_clip_z_local.las")
#     xyz = np.array([f.x, f.y, f.z], "f").T
#     rgb = np.array([f.red, f.green, f.blue])
#     rgb = (rgb / 2 ** 8).astype("u1").T
#     p = pcl.PointCloud.PointXYZRGBA.from_array(xyz, rgb)
#     normals = pcl.PointCloud.Normal()
#     p.estimate_normals_omp(normals, search_radius=0.25)
#     clouds = p.region_growing(normals,
#                               k_neighbours=20,
#                               min_cluster_size=500,
#                               max_cluster_size=10000000,
#                               smoothness_threshold_degrees=2,
#                               return_only_indices=True)
#     print(clouds)
#     return
#     for n, c in enumerate(clouds):
#         # c.view()
#         f_out = laspy.file.File("D:\Projects\cloudcompare_fix_las_write\Track_EF_clip_z_%s.las" % n, mode="w",
#                                 header=f.header)
#         f_out.x = c.x
#         f_out.y = c.y
#         f_out.z = c.z
#         f_out.red = c.r
#         f_out.green = c.g
#         f_out.blue = c.b
#         f_out.close()
#     # assert pn
