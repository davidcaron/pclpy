
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/geometry/planar_polygon.h>



template <typename PointT>
void defineGeometryPlanarPolygon(py::module &m, std::string const & suffix) {
    using Class = pcl::PlanarPolygon<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<typename pcl::PointCloud<PointT>::VectorType, Eigen::Vector4f>(), "contour"_a, "coefficients"_a);
    cls.def("setContour", &Class::setContour, "contour"_a);
    cls.def("setCoefficients", py::overload_cast<const Eigen::Vector4f &> (&Class::setCoefficients), "coefficients"_a);
    cls.def("setCoefficients", py::overload_cast<const pcl::ModelCoefficients &> (&Class::setCoefficients), "coefficients"_a);
    cls.def("getContour", py::overload_cast<> (&Class::getContour));
    cls.def("getContour", py::overload_cast<> (&Class::getContour, py::const_));
    cls.def("getCoefficients", py::overload_cast<> (&Class::getCoefficients));
    cls.def("getCoefficients", py::overload_cast<> (&Class::getCoefficients, py::const_));
        
}

void defineGeometryPlanarPolygonFunctions(py::module &m) {
}

void defineGeometryPlanarPolygonClasses(py::module &sub_module) {
}