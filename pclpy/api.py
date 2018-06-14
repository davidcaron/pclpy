import math

from . import pcl
from .view.vtk import Viewer

from . import utils
from .utils import register_point_cloud_function


@register_point_cloud_function
def extract_clusters(cloud, tolerance, min_size, max_size, merge_clusters=False):
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
def show(cloud, *other_clouds, **kwargs):
    """
    Show the point cloud using the default pclpy.Viewer
    """
    Viewer(cloud, *other_clouds, **kwargs).show()
