
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/segmentation/unary_classifier.h>



template <typename PointT>
void defineSegmentationUnaryClassifier(py::module &m, std::string const & suffix) {
    using Class = pcl::UnaryClassifier<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("train", &Class::train, "output"_a);
    cls.def("trainWithLabel", &Class::trainWithLabel, "output"_a);
    cls.def("segment", py::overload_cast<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &> (&Class::segment), "out"_a);
    cls.def("queryFeatureDistances", &Class::queryFeatureDistances, "trained_features"_a, "query_features"_a, "indi"_a, "dist"_a);
    cls.def("assignLabels", &Class::assignLabels, "indi"_a, "dist"_a, "n_feature_means"_a, "feature_threshold"_a, "out"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "input_cloud"_a);
    cls.def("setClusterSize", &Class::setClusterSize, "k"_a);
    cls.def("setNormalRadiusSearch", &Class::setNormalRadiusSearch, "param"_a);
    cls.def("setFPFHRadiusSearch", &Class::setFPFHRadiusSearch, "param"_a);
    cls.def("setLabelField", &Class::setLabelField, "l"_a);
    cls.def("setTrainedFeatures", &Class::setTrainedFeatures, "features"_a);
    cls.def("setFeatureThreshold", &Class::setFeatureThreshold, "threshold"_a);
        
}

void defineSegmentationUnaryClassifierFunctions(py::module &m) {
}

void defineSegmentationUnaryClassifierClasses(py::module &sub_module) {
    py::module sub_module_UnaryClassifier = sub_module.def_submodule("UnaryClassifier", "Submodule UnaryClassifier");
    defineSegmentationUnaryClassifier<pcl::InterestPoint>(sub_module_UnaryClassifier, "InterestPoint");
    defineSegmentationUnaryClassifier<pcl::PointDEM>(sub_module_UnaryClassifier, "PointDEM");
    defineSegmentationUnaryClassifier<pcl::PointNormal>(sub_module_UnaryClassifier, "PointNormal");
    defineSegmentationUnaryClassifier<pcl::PointSurfel>(sub_module_UnaryClassifier, "PointSurfel");
    defineSegmentationUnaryClassifier<pcl::PointWithRange>(sub_module_UnaryClassifier, "PointWithRange");
    defineSegmentationUnaryClassifier<pcl::PointWithScale>(sub_module_UnaryClassifier, "PointWithScale");
    defineSegmentationUnaryClassifier<pcl::PointWithViewpoint>(sub_module_UnaryClassifier, "PointWithViewpoint");
    defineSegmentationUnaryClassifier<pcl::PointXYZ>(sub_module_UnaryClassifier, "PointXYZ");
    defineSegmentationUnaryClassifier<pcl::PointXYZHSV>(sub_module_UnaryClassifier, "PointXYZHSV");
    defineSegmentationUnaryClassifier<pcl::PointXYZI>(sub_module_UnaryClassifier, "PointXYZI");
    defineSegmentationUnaryClassifier<pcl::PointXYZINormal>(sub_module_UnaryClassifier, "PointXYZINormal");
    defineSegmentationUnaryClassifier<pcl::PointXYZL>(sub_module_UnaryClassifier, "PointXYZL");
    defineSegmentationUnaryClassifier<pcl::PointXYZLNormal>(sub_module_UnaryClassifier, "PointXYZLNormal");
    defineSegmentationUnaryClassifier<pcl::PointXYZRGB>(sub_module_UnaryClassifier, "PointXYZRGB");
    defineSegmentationUnaryClassifier<pcl::PointXYZRGBA>(sub_module_UnaryClassifier, "PointXYZRGBA");
    defineSegmentationUnaryClassifier<pcl::PointXYZRGBL>(sub_module_UnaryClassifier, "PointXYZRGBL");
    defineSegmentationUnaryClassifier<pcl::PointXYZRGBNormal>(sub_module_UnaryClassifier, "PointXYZRGBNormal");
}