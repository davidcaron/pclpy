
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/fast_bilateral.h>



template<typename PointT>
void defineFiltersFastBilateralFilter(py::module &m, std::string const & suffix) {
    using Class = FastBilateralFilter<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("sigma_s", &Class::getSigmaS, &Class::setSigmaS);
    cls.def_property("sigma_r", &Class::getSigmaR, &Class::setSigmaR);
    cls.def("apply_filter", py::overload_cast<Filter<PointT>::PointCloud &> (&Class::applyFilter));
        
}

void defineFiltersFastBilateralClasses(py::module &sub_module) {
    py::module sub_module_FastBilateralFilter = sub_module.def_submodule("FastBilateralFilter", "Submodule FastBilateralFilter");
    defineFiltersFastBilateralFilter<PointXYZ>(sub_module_FastBilateralFilter, "PointXYZ");
    defineFiltersFastBilateralFilter<PointXYZRGB>(sub_module_FastBilateralFilter, "PointXYZRGB");
    defineFiltersFastBilateralFilter<PointXYZRGBA>(sub_module_FastBilateralFilter, "PointXYZRGBA");
}