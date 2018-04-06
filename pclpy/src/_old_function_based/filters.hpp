#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_hull.h>


namespace py = pybind11;
using namespace pybind11::literals;

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> voxel_grid(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                                 float leaf_size) {
    pcl::VoxelGrid<T> grid;
    pcl::PointCloud<T>::Ptr output_cloud (new pcl::PointCloud<T>);
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    grid.setInputCloud (input_cloud);
    grid.filter (*output_cloud);
    return output_cloud;
}

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> approximate_voxel_grid(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                                 float leaf_size) {
    pcl::ApproximateVoxelGrid<T> grid;
    pcl::PointCloud<T>::Ptr output_cloud (new pcl::PointCloud<T>);
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    grid.setInputCloud (input_cloud);
    grid.filter (*output_cloud);
    return output_cloud;
}

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> statistical_outlier_removal(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                                 float threshold,
                                                 float mean_k
                                                 ) {

    pcl::PointCloud<T>::Ptr output_cloud (new pcl::PointCloud<T>);
    pcl::StatisticalOutlierRemoval<T> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(threshold);
    sor.filter(*output_cloud);
    return output_cloud;
}

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> crop_hull(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                                boost::shared_ptr<pcl::PointCloud<T>> &clipping_cloud,
                                                int dim = 2,
                                                bool crop_outside = false
                                                ) {

    pcl::PointCloud<T>::Ptr output_cloud (new pcl::PointCloud<T>);
    pcl::CropHull<T> crop;
    crop.setInputCloud(input_cloud);
    crop.setHullCloud(clipping_cloud);
//    crop.setHullIndices(hull_indices);
    crop.setDim(dim);
    crop.setCropOutside(crop_outside);
    crop.filter(*output_cloud);
    return output_cloud;
}


