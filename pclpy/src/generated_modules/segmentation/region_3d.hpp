
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/region_3d.h>



template <typename PointT>
void defineSegmentationRegion3D(py::module &m, std::string const & suffix) {
    using Class = pcl::Region3D<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<Eigen::Vector3f, Eigen::Matrix3f, unsigned>(), "centroid"_a, "covariance"_a, "count"_a);
    cls.def("setCurvature", &Class::setCurvature, "curvature"_a);
    cls.def("getCentroid", &Class::getCentroid);
    cls.def("getCovariance", &Class::getCovariance);
    cls.def("getCount", &Class::getCount);
    cls.def("getCurvature", &Class::getCurvature);
        
}

void defineSegmentationRegion3dFunctions(py::module &m) {
}

void defineSegmentationRegion3dClasses(py::module &sub_module) {
}