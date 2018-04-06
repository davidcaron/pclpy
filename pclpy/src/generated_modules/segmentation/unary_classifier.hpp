
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/unary_classifier.h>



template <typename PointT>
void defineSegmentationUnaryClassifier(py::module &m, std::string const & suffix) {
    using Class = UnaryClassifier<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("set_cluster_size", &Class::setClusterSize);
    cls.def("set_normal_radius_search", &Class::setNormalRadiusSearch);
    cls.def("set_fpfh_radius_search", &Class::setFPFHRadiusSearch);
    cls.def("set_label_field", &Class::setLabelField);
    cls.def("set_trained_features", &Class::setTrainedFeatures);
    cls.def("set_feature_threshold", &Class::setFeatureThreshold);
    cls.def("segment", py::overload_cast<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &> (&Class::segment));
    cls.def("train", &Class::train);
    cls.def("train_with_label", &Class::trainWithLabel);
    cls.def("query_feature_distances", &Class::queryFeatureDistances);
    cls.def("assign_labels", &Class::assignLabels);
        
}

void defineSegmentationUnaryClassifierClasses(py::module &sub_module) {
    py::module sub_module_UnaryClassifier = sub_module.def_submodule("UnaryClassifier", "Submodule UnaryClassifier");
    defineSegmentationUnaryClassifier<InterestPoint>(sub_module_UnaryClassifier, "InterestPoint");
    defineSegmentationUnaryClassifier<PointDEM>(sub_module_UnaryClassifier, "PointDEM");
    defineSegmentationUnaryClassifier<PointNormal>(sub_module_UnaryClassifier, "PointNormal");
    defineSegmentationUnaryClassifier<PointSurfel>(sub_module_UnaryClassifier, "PointSurfel");
    defineSegmentationUnaryClassifier<PointWithRange>(sub_module_UnaryClassifier, "PointWithRange");
    defineSegmentationUnaryClassifier<PointWithScale>(sub_module_UnaryClassifier, "PointWithScale");
    defineSegmentationUnaryClassifier<PointWithViewpoint>(sub_module_UnaryClassifier, "PointWithViewpoint");
    defineSegmentationUnaryClassifier<PointXYZ>(sub_module_UnaryClassifier, "PointXYZ");
    defineSegmentationUnaryClassifier<PointXYZHSV>(sub_module_UnaryClassifier, "PointXYZHSV");
    defineSegmentationUnaryClassifier<PointXYZI>(sub_module_UnaryClassifier, "PointXYZI");
    defineSegmentationUnaryClassifier<PointXYZINormal>(sub_module_UnaryClassifier, "PointXYZINormal");
    defineSegmentationUnaryClassifier<PointXYZL>(sub_module_UnaryClassifier, "PointXYZL");
    defineSegmentationUnaryClassifier<PointXYZLNormal>(sub_module_UnaryClassifier, "PointXYZLNormal");
    defineSegmentationUnaryClassifier<PointXYZRGB>(sub_module_UnaryClassifier, "PointXYZRGB");
    defineSegmentationUnaryClassifier<PointXYZRGBA>(sub_module_UnaryClassifier, "PointXYZRGBA");
    defineSegmentationUnaryClassifier<PointXYZRGBL>(sub_module_UnaryClassifier, "PointXYZRGBL");
    defineSegmentationUnaryClassifier<PointXYZRGBNormal>(sub_module_UnaryClassifier, "PointXYZRGBNormal");
}