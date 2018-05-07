
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

#pragma warning(disable : 4267)
#include <pcl/recognition/ransac_based/model_library.h>

using namespace pcl::recognition;


void defineRecognitionModelLibrary(py::module &m) {
    using Class = pcl::recognition::ModelLibrary;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudN = Class::PointCloudN;
    using node_data_pair_list = Class::node_data_pair_list;
    using HashTableCell = Class::HashTableCell;
    using HashTable = Class::HashTable;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "ModelLibrary");
    cls.def(py::init<float, float, float>(), "pair_width"_a, "voxel_size"_a, "max_coplanarity_angle"_a=3.0f*AUX_DEG_TO_RADIANS);
    cls.def("removeAllModels", &Class::removeAllModels);
    cls.def("ignoreCoplanarPointPairsOn", &Class::ignoreCoplanarPointPairsOn);
    cls.def("ignoreCoplanarPointPairsOff", &Class::ignoreCoplanarPointPairsOff);
    cls.def("addModel", &Class::addModel, "points"_a, "normals"_a, "object_name"_a, "frac_of_points_for_registration"_a, "user_data"_a=NULL);
    cls.def("setMaxCoplanarityAngleDegrees", &Class::setMaxCoplanarityAngleDegrees, "max_coplanarity_angle_degrees"_a);
    cls.def("getHashTable", &Class::getHashTable);
    cls.def("getModel", &Class::getModel, "name"_a);
    cls.def("getModels", &Class::getModels);
}

void defineRecognitionModelLibraryFunctions(py::module &m) {
}

void defineRecognitionModelLibraryClasses(py::module &sub_module) {
    defineRecognitionModelLibrary(sub_module);
}