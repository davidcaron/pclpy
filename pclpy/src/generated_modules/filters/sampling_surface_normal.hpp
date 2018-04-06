
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/sampling_surface_normal.h>



template<typename PointT>
void defineFiltersSamplingSurfaceNormal(py::module &m, std::string const & suffix) {
    using Class = SamplingSurfaceNormal<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("sample", &Class::getSample, &Class::setSample);
    cls.def_property("seed", &Class::getSeed, &Class::setSeed);
    cls.def_property("ratio", &Class::getRatio, &Class::setRatio);
        
}

void defineFiltersSamplingSurfaceNormalClasses(py::module &sub_module) {
    py::module sub_module_SamplingSurfaceNormal = sub_module.def_submodule("SamplingSurfaceNormal", "Submodule SamplingSurfaceNormal");
    defineFiltersSamplingSurfaceNormal<PointNormal>(sub_module_SamplingSurfaceNormal, "PointNormal");
    defineFiltersSamplingSurfaceNormal<PointXYZINormal>(sub_module_SamplingSurfaceNormal, "PointXYZINormal");
    defineFiltersSamplingSurfaceNormal<PointXYZRGBNormal>(sub_module_SamplingSurfaceNormal, "PointXYZRGBNormal");
}