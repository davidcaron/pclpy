
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/region_3d.h>



template <typename PointT>
void defineSegmentationRegion3D(py::module &m, std::string const & suffix) {
    using Class = Region3D<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<Eigen::Vector3f, Eigen::Matrix3f, unsigned>(), "centroid"_a, "covariance"_a, "count"_a);
    cls.def_property("curvature", &Class::getCurvature, &Class::setCurvature);
        
}

void defineSegmentationRegion3dClasses(py::module &sub_module) {
}