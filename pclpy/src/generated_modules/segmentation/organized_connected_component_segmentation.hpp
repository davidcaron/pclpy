
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/organized_connected_component_segmentation.h>



template <typename PointT, typename PointLT>
void defineSegmentationOrganizedConnectedComponentSegmentation(py::module &m, std::string const & suffix) {
    using Class = OrganizedConnectedComponentSegmentation<PointT, PointLT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudL = Class::PointCloudL;
    using PointCloudLPtr = Class::PointCloudLPtr;
    using PointCloudLConstPtr = Class::PointCloudLConstPtr;
    using Comparator = Class::Comparator;
    using ComparatorPtr = Class::ComparatorPtr;
    using ComparatorConstPtr = Class::ComparatorConstPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Class::ComparatorConstPtr>(), "compare"_a);
    cls.def_property("comparator", &Class::getComparator, &Class::setComparator);
    cls.def("segment", py::overload_cast<pcl::PointCloud<PointLT> &, std::vector<pcl::PointIndices> &> (&Class::segment, py::const_));
    cls.def("find_labeled_region_boundary", &Class::findLabeledRegionBoundary);
        
}

void defineSegmentationOrganizedConnectedComponentSegmentationClasses(py::module &sub_module) {
    py::module sub_module_OrganizedConnectedComponentSegmentation = sub_module.def_submodule("OrganizedConnectedComponentSegmentation", "Submodule OrganizedConnectedComponentSegmentation");
    defineSegmentationOrganizedConnectedComponentSegmentation<InterestPoint, Label>(sub_module_OrganizedConnectedComponentSegmentation, "InterestPoint_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointDEM, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointDEM_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointNormal, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointNormal_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointSurfel, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointSurfel_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointWithRange, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointWithRange_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointWithScale, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointWithScale_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointWithViewpoint, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointWithViewpoint_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZ, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZ_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZHSV, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZHSV_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZI, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZI_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZINormal, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZINormal_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZL, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZL_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZLNormal, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZLNormal_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZRGB, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZRGB_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZRGBA, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZRGBA_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZRGBL, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZRGBL_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<PointXYZRGBNormal, Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZRGBNormal_Label");
}