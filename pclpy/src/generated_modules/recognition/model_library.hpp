
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/model_library.h>

using namespace pcl::recognition;


void defineRecognitionModelLibrary(py::module &m) {
    using Class = recognition::ModelLibrary;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudN = Class::PointCloudN;
    using node_data_pair_list = Class::node_data_pair_list;
    using HashTableCell = Class::HashTableCell;
    using HashTable = Class::HashTable;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ModelLibrary");
    cls.def(py::init<float, float, float>(), "pair_width"_a, "voxel_size"_a, "max_coplanarity_angle"_a=3.0f*AUX_DEG_TO_RADIANS);
    cls.def("set_max_coplanarity_angle_degrees", &Class::setMaxCoplanarityAngleDegrees);
    cls.def("remove_all_models", &Class::removeAllModels);
    cls.def("ignore_coplanar_point_pairs_on", &Class::ignoreCoplanarPointPairsOn);
    cls.def("ignore_coplanar_point_pairs_off", &Class::ignoreCoplanarPointPairsOff);
    cls.def("add_model", &Class::addModel);
}

void defineRecognitionModelLibraryClasses(py::module &sub_module) {
    defineRecognitionModelLibrary(sub_module);
}