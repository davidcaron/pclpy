
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/gfpfh.h>



template <typename PointInT, typename PointLT, typename PointOutT>
void defineFeaturesGFPFHEstimation(py::module &m, std::string const & suffix) {
    using Class = GFPFHEstimation<PointInT, PointLT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, FeatureFromLabels<PointInT,PointLT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("octree_leaf_size", &Class::getOctreeLeafSize, &Class::setOctreeLeafSize);
    cls.def_property("number_of_classes", &Class::getNumberOfClasses, &Class::setNumberOfClasses);
    cls.def("empty_label", &Class::emptyLabel);
    cls.def("descriptor_size", &Class::descriptorSize);
    cls.def("compute", &Class::compute);
        
}

void defineFeaturesGfpfhClasses(py::module &sub_module) {
    py::module sub_module_GFPFHEstimation = sub_module.def_submodule("GFPFHEstimation", "Submodule GFPFHEstimation");
    defineFeaturesGFPFHEstimation<PointXYZ, PointXYZL, GFPFHSignature16>(sub_module_GFPFHEstimation, "PointXYZ_PointXYZL_GFPFHSignature16");
    defineFeaturesGFPFHEstimation<PointXYZI, PointXYZL, GFPFHSignature16>(sub_module_GFPFHEstimation, "PointXYZI_PointXYZL_GFPFHSignature16");
    defineFeaturesGFPFHEstimation<PointXYZL, PointXYZL, GFPFHSignature16>(sub_module_GFPFHEstimation, "PointXYZL_PointXYZL_GFPFHSignature16");
    defineFeaturesGFPFHEstimation<PointXYZRGBA, PointXYZL, GFPFHSignature16>(sub_module_GFPFHEstimation, "PointXYZRGBA_PointXYZL_GFPFHSignature16");
}