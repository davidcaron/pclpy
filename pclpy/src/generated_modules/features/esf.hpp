
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/esf.h>



template <typename PointInT,  typename PointOutT = pcl::ESFSignature640>
void defineFeaturesESFEstimation(py::module &m, std::string const & suffix) {
    using Class = ESFEstimation<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("compute", &Class::compute);
        
}

void defineFeaturesEsfClasses(py::module &sub_module) {
    py::module sub_module_ESFEstimation = sub_module.def_submodule("ESFEstimation", "Submodule ESFEstimation");
    defineFeaturesESFEstimation<PointNormal, ESFSignature640>(sub_module_ESFEstimation, "PointNormal_ESFSignature640");
    defineFeaturesESFEstimation<PointXYZ, ESFSignature640>(sub_module_ESFEstimation, "PointXYZ_ESFSignature640");
    defineFeaturesESFEstimation<PointXYZI, ESFSignature640>(sub_module_ESFEstimation, "PointXYZI_ESFSignature640");
    defineFeaturesESFEstimation<PointXYZRGBA, ESFSignature640>(sub_module_ESFEstimation, "PointXYZRGBA_ESFSignature640");
}