#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

namespace py = pybind11;
using namespace pybind11::literals;

template <typename T, typename U>
void mls_compute_func(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                      boost::shared_ptr<pcl::PointCloud<U>> &output_cloud,
                      float search_radius,
                      int polynomial_order,
                      bool polynomial_fit,
                      bool compute_normals) {
    pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
    pcl::MovingLeastSquares<T, U> mls;
    mls.setComputeNormals(compute_normals);
    mls.setInputCloud (input_cloud);
    mls.setPolynomialFit (polynomial_fit);
    mls.setPolynomialOrder (polynomial_order);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (search_radius);
    mls.process (*output_cloud);
}

template <typename T, typename U>
void mls_compute_func_omp(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                          boost::shared_ptr<pcl::PointCloud<U>> &output_cloud,
                          float search_radius,
                          int polynomial_order,
                          bool polynomial_fit,
                          bool compute_normals) {
    pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
    pcl::MovingLeastSquaresOMP<T, U> mls;
    mls.setComputeNormals(compute_normals);
    mls.setInputCloud (input_cloud);
    mls.setPolynomialFit (polynomial_fit);
    mls.setPolynomialOrder (polynomial_order);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (search_radius);
    mls.process (*output_cloud);
}

template <typename T, typename U>
static void defineMls(py::class_<boost::shared_ptr<pcl::PointCloud<T>>> &cls) {
    cls.def("mls", &mls_compute_func<T, U>,
                                       "output_cloud"_a,
                                       "search_radius"_a,
                                       "polynomial_order"_a=2,
                                       "polynomial_fit"_a=false,
                                       "compute_normals"_a=false, "mls!");
    cls.def("mls_omp", &mls_compute_func_omp<T, U>,
                                       "output_cloud"_a,
                                       "search_radius"_a,
                                       "polynomial_order"_a=2,
                                       "polynomial_fit"_a=false,
                                       "compute_normals"_a=false, "mls!");
}

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> concave_hull (boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                                    float alpha,
                                                    int dim = 2) {
    pcl::PointCloud<T>::Ptr cloud_hull (new pcl::PointCloud<T>);
    pcl::ConcaveHull<T> chull;
    chull.setInputCloud (input_cloud);
    chull.setAlpha(alpha);
    chull.setDimension(dim);
    chull.reconstruct(*cloud_hull);
    return cloud_hull;
}
template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> convex_hull (boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                                   int dim = 2) {
    pcl::PointCloud<T>::Ptr cloud_hull (new pcl::PointCloud<T>);
    pcl::ConvexHull<T> chull;
    chull.setInputCloud (input_cloud);
    chull.setDimension(dim);
    chull.reconstruct(*cloud_hull);
    return cloud_hull;
}
