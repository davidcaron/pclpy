
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/surface_normal_modality.h>



void defineRecognitionLINEMOD_OrientationMap(py::module &m) {
    using Class = LINEMOD_OrientationMap;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "LINEMOD_OrientationMap");
    cls.def(py::init<>());
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("resize", &Class::resize);
}

void defineRecognitionQuantizedNormalLookUpTable(py::module &m) {
    using Class = QuantizedNormalLookUpTable;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "QuantizedNormalLookUpTable");
    cls.def(py::init<>());
    cls.def_readonly("range_x", &Class::range_x);
    cls.def_readonly("range_y", &Class::range_y);
    cls.def_readonly("range_z", &Class::range_z);
    cls.def_readonly("offset_x", &Class::offset_x);
    cls.def_readonly("offset_y", &Class::offset_y);
    cls.def_readonly("offset_z", &Class::offset_z);
    cls.def_readonly("size_x", &Class::size_x);
    cls.def_readonly("size_y", &Class::size_y);
    cls.def_readonly("size_z", &Class::size_z);
    cls.def_readonly("lut", &Class::lut);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("initialize_lut", &Class::initializeLUT);
}

template <typename PointInT>
void defineRecognitionSurfaceNormalModality(py::module &m, std::string const & suffix) {
    using Class = SurfaceNormalModality<PointInT>;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, QuantizableModality, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_spreading_size", &Class::setSpreadingSize);
    cls.def("set_variable_feature_nr", &Class::setVariableFeatureNr);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("extract_features", &Class::extractFeatures);
    cls.def("extract_all_features", &Class::extractAllFeatures);
    cls.def("process_input_data", &Class::processInputData);
    cls.def("process_input_data_from_filtered", &Class::processInputDataFromFiltered);
    cls.def("get_surface_normals", py::overload_cast<> (&Class::getSurfaceNormals));
    cls.def("get_surface_normals", py::overload_cast<> (&Class::getSurfaceNormals, py::const_));
        
}

void defineRecognitionSurfaceNormalModalityClasses(py::module &sub_module) {
    defineRecognitionLINEMOD_OrientationMap(sub_module);
    defineRecognitionQuantizedNormalLookUpTable(sub_module);
}