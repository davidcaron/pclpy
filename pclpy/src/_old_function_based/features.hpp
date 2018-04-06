#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <pcl/features/normal_3d_omp.h>

namespace py = pybind11;
using namespace pybind11::literals;


template <typename T, typename U>
boost::shared_ptr<pcl::PointCloud<U>> normals_compute_func(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                      boost::shared_ptr<pcl::PointCloud<U>> &output_cloud,
                      float search_radius) {
    pcl::NormalEstimation<T,U> nest;
    nest.setRadiusSearch (search_radius);
    nest.setInputCloud (input_cloud);
    nest.compute (*output_cloud);
    return output_cloud;
}

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> normals_compute_func_inplace(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                      float search_radius) {
    return normals_compute_func(input_cloud, input_cloud, search_radius);
}

template <typename T, typename U>
boost::shared_ptr<pcl::PointCloud<U>> normals_compute_func_omp(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                      boost::shared_ptr<pcl::PointCloud<U>> &output_cloud,
                      float search_radius) {
    pcl::NormalEstimationOMP<T,U> nest;
    nest.setRadiusSearch (search_radius);
    nest.setInputCloud (input_cloud);
    nest.compute (*output_cloud);
    return output_cloud;
}

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> normals_compute_func_omp_inplace(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                      float search_radius) {
    return normals_compute_func_omp(input_cloud, input_cloud, search_radius);
}

template <typename T, typename U>
static void defineNormalEstimation(py::class_<boost::shared_ptr<pcl::PointCloud<T>>> &cls) {
    cls.def("estimate_normals", &normals_compute_func<T, U>,
                                       "output_cloud"_a,
                                       "search_radius"_a);
    cls.def("estimate_normals_omp", &normals_compute_func_omp<T, U>,
                                       "output_cloud"_a,
                                       "search_radius"_a);
}

template <typename T>
static void defineNormalEstimationInplace(py::class_<boost::shared_ptr<pcl::PointCloud<T>>> &cls) {
    cls.def("estimate_normals", &normals_compute_func_inplace<T>,
                                       "search_radius"_a);
    cls.def("estimate_normals_omp", &normals_compute_func_omp_inplace<T>,
                                       "search_radius"_a);
}