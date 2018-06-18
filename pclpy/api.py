import math

import numpy as np

from . import pcl
from .view.vtk import Viewer

from . import utils
from .utils import register_point_cloud_function


@register_point_cloud_function
def extract_clusters(cloud, tolerance, min_size, max_size, merge_clusters=False):
    """
    Compute an EuclideanClusterExtraction on an input point cloud
    :param cloud: input point cloud
    :param tolerance: distance tolerance
    :param min_size: minimum cluster size
    :param max_size: maximum cluster size
    :param merge_clusters: set to True to output a single cloud or False to output a pcl.vectors.PointIndices
    :return: either an instance of pcl.PointCloud.* or pcl.vectors.PointIndices depending on merge_clusters
    """
    pc_type = utils.get_point_cloud_type(cloud)
    ext = getattr(pcl.segmentation.EuclideanClusterExtraction, pc_type)()
    ext.setInputCloud(cloud)
    ext.setClusterTolerance(tolerance)
    ext.setMinClusterSize(min_size)
    ext.setMaxClusterSize(max_size)
    vector_indices = pcl.vectors.PointIndices()
    ext.extract(vector_indices)

    if merge_clusters:
        indices = pcl.vectors.Int()
        for i in vector_indices:
            indices.extend(i.indices)
        return getattr(pcl.PointCloud, pc_type)(cloud, indices)

    return vector_indices


@register_point_cloud_function
def region_growing(cloud,
                   normals,
                   n_neighbours,
                   min_size,
                   max_size,
                   smooth_threshold,
                   curvature_threshold,
                   residual_threshold
                   ):
    """
    Compute a region growing segmentation
    :param cloud: input point cloud
    :param normals: input normals (can be the same as input point cloud)
    :param n_neighbours: number of neighbors to use
    :param min_size: minimum cluster size
    :param max_size: maximum cluster size
    :param smooth_threshold: passed to setSmoothnessThreshold
    :param curvature_threshold: passed to setCurvatureThreshold
    :param residual_threshold: passed to setResidualThreshold
    :return: a vector of pcl.PointIndices (pcl.vectors.PointIndices)
    """
    pc_type = utils.get_point_cloud_type(cloud, normals)
    rg = getattr(pcl.segmentation.RegionGrowing, pc_type)()

    rg.setInputCloud(cloud)
    rg.setInputNormals(normals)

    rg.setMinClusterSize(min_size)
    rg.setMaxClusterSize(max_size)
    rg.setNumberOfNeighbours(n_neighbours)
    rg.setSmoothnessThreshold(smooth_threshold / 180 * math.pi)
    rg.setCurvatureThreshold(curvature_threshold)
    rg.setResidualThreshold(residual_threshold)
    vector_indices = pcl.vectors.PointIndices()
    rg.extract(vector_indices)
    return vector_indices


@register_point_cloud_function
def moving_least_squares(cloud,
                         search_radius,
                         output_cloud=None,
                         compute_normals=False,
                         polynomial_fit=True,
                         polynomial_order=2,
                         num_threads=1,
                         ):
    """
    Compute a moving least squares on the input cloud
    :param cloud: input point cloud
    :param search_radius: radius search distance
    :param output_cloud: optional point cloud to compute into
    :param compute_normals: boolean, set to compute normals
    :param polynomial_fit: boolean, set to compute using a polynomial function instead of a plane
    :param polynomial_order: order of the polynomial function to fit
    :param num_threads: optional number of threads to use
    :return: a smoothed point cloud
    """
    if output_cloud is None:
        if compute_normals:
            output_cloud = pcl.PointCloud.PointNormal()
        else:
            cloud_type = utils.get_point_cloud_type(cloud)
            output_cloud = getattr(pcl.PointCloud, cloud_type)()

    pc_type = utils.get_point_cloud_type(cloud, output_cloud)
    if num_threads == 1:
        mls = getattr(pcl.surface.MovingLeastSquares, pc_type)()
    else:
        mls = getattr(pcl.surface.MovingLeastSquaresOMP, pc_type)(num_threads)

    mls.setInputCloud(cloud)
    mls.setComputeNormals(compute_normals)
    mls.setPolynomialFit(polynomial_fit)
    mls.setPolynomialOrder(polynomial_order)
    mls.setSearchRadius(search_radius)
    mls.process(output_cloud)
    return output_cloud


@register_point_cloud_function
def radius_outlier_removal(cloud,
                           search_radius,
                           min_neighbors,
                           negative=False,
                           indices=None,
                           ):
    """
    Compute a radius outlier removal
    :param cloud: input point cloud
    :param search_radius: radius search distance
    :param min_neighbors: minimum number of neighbors
    :param negative: passed to setNegative
    :param indices: optional indices of the input cloud to use
    :return:
    """
    pc_type = utils.get_point_cloud_type(cloud)
    ror_filter = getattr(pcl.filters.RadiusOutlierRemoval, pc_type)()
    ror_filter.setInputCloud(cloud)
    ror_filter.setRadiusSearch(search_radius)
    ror_filter.setNegative(negative)
    ror_filter.setMinNeighborsInRadius(min_neighbors)
    if indices is not None:
        ror_filter.setIndices(indices)
    output = getattr(pcl.PointCloud, pc_type)()
    ror_filter.filter(output)
    return output


@register_point_cloud_function
def compute_normals(cloud, radius=None, k=None, indices=None, num_threads=1, output_cloud=None):
    """
    Compute normals for a point cloud
    :param cloud: input point cloud
    :param radius: radius search distance
    :param k: use k nearest neighbors
    :param indices: optional indices of the input cloud to use
    :param num_threads: number of threads to do the computation
    :param output_cloud: optional point cloud to compute the normals into
    :return: a point cloud with normals
    """
    if output_cloud is None:
        output_cloud = pcl.PointCloud.Normal()
    pc_type = utils.get_point_cloud_type(cloud, output_cloud)

    if num_threads == 1:
        normals_estimation = getattr(pcl.features.NormalEstimation, pc_type)()
    else:
        normals_estimation = getattr(pcl.features.NormalEstimationOMP, pc_type)(num_threads)
    normals_estimation.setInputCloud(cloud)
    if radius is not None:
        normals_estimation.setRadiusSearch(radius)
    if k is not None:
        normals_estimation.setKSearch(k)
    if indices is not None:
        normals_estimation.setIndices(indices)

    normals_estimation.compute(output_cloud)
    return output_cloud


@register_point_cloud_function
def octree_voxel_centroid(cloud, resolution, epsilon=None):
    """
    Subsample the input cloud to the octree's centroids
    :param cloud: input point cloud
    :param resolution: float size of the smallest leaf
    :param epsilon: epsilon precision for nearest neighbor searches, passed to setEpsilon
    :return: pcl.PointCloud.* same type as input
    """
    pc_type = utils.get_point_cloud_type(cloud)
    vox = getattr(pcl.octree.OctreePointCloudVoxelCentroid, pc_type)(resolution)
    vox.setInputCloud(cloud)
    vox.addPointsFromInputCloud()
    if epsilon:
        vox.setEpsilon(epsilon)
    centroids = getattr(pcl.PointCloud, pc_type)()
    vox.getVoxelCentroids(centroids.points)
    return centroids


@register_point_cloud_function
def fit(cloud, model, distance, method=pcl.sample_consensus.SAC_RANSAC, indices=None, optimize=True):
    """
    Fit a model to a cloud using a sample consensus method
    :param cloud: input point cloud
    :param model: str (ex.: 'line', 'sphere', ...) or an instance of pcl.sample_consensus.SacModel
    :param distance: distance threshold
    :param method: SAC method to use
    :param indices: optional indices of the input cloud to use
    :param optimize: passed to setOptimizeCoefficients
    :return: (inliers: pcl.PointIndices, coefficients: pcl.ModelCoefficients)
    """
    models = [
        "circle2d",
        "circle3d",
        "cone",
        "cylinder",               # needs normals
        "line",                   # needs normals
        "normal_parallel_plane",  # needs normals
        "normal_plane",           # needs normals
        "normal_sphere",          # needs normals
        "parallel_line",
        "parallel_lines",
        "parallel_plane",
        "perpendicular_plane",
        "plane",
        "registration",
        "registration_2d",
        "sphere",
        "stick",
        "torus",                  # needs normals
    ]
    if isinstance(model, str):
        model = model.lower()
        for model_name in models:
            if model_name == model:
                model = getattr(pcl.sample_consensus, "SACMODEL_" + model_name.upper())
                break

    if not isinstance(model, pcl.sample_consensus.SacModel):  # pcl.sample_consensus.SACMODEL_*
        message = ("Unrecognized model: %s. Must be either a string "
                   "or an enum from pcl.sample_consensus.SACMODEL_*")
        raise ValueError(message)

    pc_type = utils.get_point_cloud_type(cloud)
    seg = getattr(pcl.segmentation.SACSegmentation, pc_type)()
    pcl.segmentation
    seg.setOptimizeCoefficients(optimize)
    seg.setModelType(model)
    seg.setMethodType(method)
    seg.setDistanceThreshold(distance)
    seg.setInputCloud(cloud)

    if indices is not None:
        if isinstance(indices, np.ndarray):
            indices = pcl.vectors.Int(indices)
        seg.setIndices(indices)
    coefficients = pcl.ModelCoefficients()
    inliers = pcl.PointIndices()
    seg.segment(inliers, coefficients)
    return inliers, coefficients


@register_point_cloud_function
def show(cloud, *other_clouds, **kwargs):
    """
    Show the point cloud using the default pclpy.Viewer
    """
    Viewer(cloud, *other_clouds, **kwargs).show()
