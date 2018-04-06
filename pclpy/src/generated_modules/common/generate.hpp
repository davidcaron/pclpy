
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/generate.h>

using namespace pcl::common;


template <typename PointT, typename GeneratorT>
void defineCommonCloudGenerator(py::module &m, std::string const & suffix) {
    using Class = common::CloudGenerator<PointT, GeneratorT>;
    using GeneratorParameters = Class::GeneratorParameters;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<Class::GeneratorParameters>(), "params"_a);
    cls.def(py::init<Class::GeneratorParameters, Class::GeneratorParameters, Class::GeneratorParameters>(), "x_params"_a, "y_params"_a, "z_params"_a);
    cls.def("set_parameters", &Class::setParameters);
    cls.def_property("parameters_for_x", &Class::getParametersForX, &Class::setParametersForX);
    cls.def_property("parameters_for_y", &Class::getParametersForY, &Class::setParametersForY);
    cls.def_property("parameters_for_z", &Class::getParametersForZ, &Class::setParametersForZ);
    cls.def("fill", py::overload_cast<pcl::PointCloud<PointT> &> (&Class::fill));
    cls.def("fill", py::overload_cast<int, int, pcl::PointCloud<PointT> &> (&Class::fill));
        
}



void defineCommonGenerateClasses(py::module &sub_module) {
}