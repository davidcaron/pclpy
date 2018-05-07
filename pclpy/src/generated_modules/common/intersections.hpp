
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/common/intersections.h>



void defineCommonIntersectionsFunctions1(py::module &m) {
    m.def("lineWithLineIntersection", py::overload_cast<const Eigen::VectorXf &, const Eigen::VectorXf &, Eigen::Vector4f &, double> (&pcl::lineWithLineIntersection), "line_a"_a, "line_b"_a, "point"_a, "sqr_eps"_a=1e-4);
    m.def("lineWithLineIntersection", py::overload_cast<const pcl::ModelCoefficients &, const pcl::ModelCoefficients &, Eigen::Vector4f &, double> (&pcl::lineWithLineIntersection), "line_a"_a, "line_b"_a, "point"_a, "sqr_eps"_a=1e-4);
    m.def("planeWithPlaneIntersection", py::overload_cast<const Eigen::Vector4f &, const Eigen::Vector4f &, Eigen::VectorXf &, double> (&pcl::planeWithPlaneIntersection), "plane_a"_a, "plane_b"_a, "line"_a, "angular_tolerance"_a=0.1);
    m.def("planeWithPlaneIntersection", py::overload_cast<const Eigen::Vector4d &, const Eigen::Vector4d &, Eigen::VectorXd &, double> (&pcl::planeWithPlaneIntersection), "plane_a"_a, "plane_b"_a, "line"_a, "angular_tolerance"_a=0.1);
    m.def("threePlanesIntersection", py::overload_cast<const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4f &, Eigen::Vector3f &, double> (&pcl::threePlanesIntersection), "plane_a"_a, "plane_b"_a, "plane_c"_a, "intersection_point"_a, "determinant_tolerance"_a=1e-6);
    m.def("threePlanesIntersection", py::overload_cast<const Eigen::Vector4d &, const Eigen::Vector4d &, const Eigen::Vector4d &, Eigen::Vector3d &, double> (&pcl::threePlanesIntersection), "plane_a"_a, "plane_b"_a, "plane_c"_a, "intersection_point"_a, "determinant_tolerance"_a=1e-6);
}

template <typename Scalar>
void defineCommonIntersectionsFunctions2(py::module &m) {
    m.def("planeWithPlaneIntersection", py::overload_cast<const Eigen::Matrix<Scalar, 4, 1> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &, double> (&pcl::planeWithPlaneIntersection<Scalar>), "plane_a"_a, "plane_b"_a, "line"_a, "angular_tolerance"_a=0.1);
    m.def("threePlanesIntersection", py::overload_cast<const Eigen::Matrix<Scalar, 4, 1> &, const Eigen::Matrix<Scalar, 4, 1> &, const Eigen::Matrix<Scalar, 4, 1> &, Eigen::Matrix<Scalar, 3, 1> &, double> (&pcl::threePlanesIntersection<Scalar>), "plane_a"_a, "plane_b"_a, "plane_c"_a, "intersection_point"_a, "determinant_tolerance"_a=1e-6);
}

void defineCommonIntersectionsFunctions(py::module &m) {
    defineCommonIntersectionsFunctions1(m);
    defineCommonIntersectionsFunctions2<float>(m);
}

void defineCommonIntersectionsClasses(py::module &sub_module) {
    defineCommonIntersectionsFunctions(sub_module);
}