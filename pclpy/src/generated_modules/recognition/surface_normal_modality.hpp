
#include <pcl/recognition/surface_normal_modality.h>



void defineRecognitionLINEMOD_OrientationMap(py::module &m) {
    using Class = pcl::LINEMOD_OrientationMap;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LINEMOD_OrientationMap");
    cls.def(py::init<>());
    cls.def("resize", &Class::resize, "width"_a, "height"_a, "value"_a);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
}

void defineRecognitionQuantizedNormalLookUpTable(py::module &m) {
    using Class = pcl::QuantizedNormalLookUpTable;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "QuantizedNormalLookUpTable");
    cls.def(py::init<>());
    cls.def_readwrite("range_x", &Class::range_x);
    cls.def_readwrite("range_y", &Class::range_y);
    cls.def_readwrite("range_z", &Class::range_z);
    cls.def_readwrite("offset_x", &Class::offset_x);
    cls.def_readwrite("offset_y", &Class::offset_y);
    cls.def_readwrite("offset_z", &Class::offset_z);
    cls.def_readwrite("size_x", &Class::size_x);
    cls.def_readwrite("size_y", &Class::size_y);
    cls.def_readwrite("size_z", &Class::size_z);
    cls.def_readwrite("lut", &Class::lut);
    cls.def("initializeLUT", &Class::initializeLUT, "range_x_arg"_a, "range_y_arg"_a, "range_z_arg"_a);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
}

template <typename PointInT>
void defineRecognitionSurfaceNormalModality(py::module &m, std::string const & suffix) {
    using Class = pcl::SurfaceNormalModality<PointInT>;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::QuantizableModality, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("extractFeatures", &Class::extractFeatures, "mask"_a, "nr_features"_a, "modality_index"_a, "features"_a);
    cls.def("extractAllFeatures", &Class::extractAllFeatures, "mask"_a, "nr_features"_a, "modality_index"_a, "features"_a);
    cls.def("processInputData", &Class::processInputData);
    cls.def("processInputDataFromFiltered", &Class::processInputDataFromFiltered);
    cls.def("setSpreadingSize", &Class::setSpreadingSize, "spreading_size"_a);
    cls.def("setVariableFeatureNr", &Class::setVariableFeatureNr, "enabled"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getSurfaceNormals", py::overload_cast<> (&Class::getSurfaceNormals));
    cls.def("getSurfaceNormals", py::overload_cast<> (&Class::getSurfaceNormals, py::const_));
    cls.def("getQuantizedMap", &Class::getQuantizedMap);
    cls.def("getSpreadedQuantizedMap", &Class::getSpreadedQuantizedMap);
    cls.def("getOrientationMap", &Class::getOrientationMap);
        
}

void defineRecognitionSurfaceNormalModalityFunctions(py::module &m) {
}

void defineRecognitionSurfaceNormalModalityClasses(py::module &sub_module) {
    defineRecognitionLINEMOD_OrientationMap(sub_module);
    defineRecognitionQuantizedNormalLookUpTable(sub_module);
}