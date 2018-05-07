
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/organized_connected_component_segmentation.h>



template <typename PointT, typename PointLT>
void defineSegmentationOrganizedConnectedComponentSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::OrganizedConnectedComponentSegmentation<PointT, PointLT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudL = Class::PointCloudL;
    using PointCloudLPtr = Class::PointCloudLPtr;
    using PointCloudLConstPtr = Class::PointCloudLConstPtr;
    using Comparator = Class::Comparator;
    using ComparatorPtr = Class::ComparatorPtr;
    using ComparatorConstPtr = Class::ComparatorConstPtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<ComparatorConstPtr>(), "compare"_a);
    cls.def("segment", py::overload_cast<pcl::PointCloud<PointLT> &, std::vector<pcl::PointIndices> &> (&Class::segment, py::const_), "labels"_a, "label_indices"_a);
    cls.def_static("findLabeledRegionBoundary", &Class::findLabeledRegionBoundary, "start_idx"_a, "labels"_a, "boundary_indices"_a);
    cls.def("setComparator", &Class::setComparator, "compare"_a);
    cls.def("getComparator", &Class::getComparator);
        
}

void defineSegmentationOrganizedConnectedComponentSegmentationFunctions(py::module &m) {
}

void defineSegmentationOrganizedConnectedComponentSegmentationClasses(py::module &sub_module) {
    py::module sub_module_OrganizedConnectedComponentSegmentation = sub_module.def_submodule("OrganizedConnectedComponentSegmentation", "Submodule OrganizedConnectedComponentSegmentation");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::InterestPoint, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "InterestPoint_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointDEM, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointDEM_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointNormal, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointNormal_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointSurfel, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointSurfel_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointWithRange, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointWithRange_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointWithScale, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointWithScale_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointWithViewpoint, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointWithViewpoint_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZ, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZ_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZHSV, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZHSV_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZI, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZI_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZINormal, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZINormal_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZL, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZL_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZLNormal, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZLNormal_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZRGB, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZRGB_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZRGBA_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZRGBL, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZRGBL_Label");
    defineSegmentationOrganizedConnectedComponentSegmentation<pcl::PointXYZRGBNormal, pcl::Label>(sub_module_OrganizedConnectedComponentSegmentation, "PointXYZRGBNormal_Label");
}