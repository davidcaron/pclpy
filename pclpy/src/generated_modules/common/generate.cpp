
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/common/generate.h>

using namespace pcl::common;


template <typename PointT, typename GeneratorT>
void defineCommonCloudGenerator(py::module &m, std::string const & suffix) {
    using Class = pcl::common::CloudGenerator<PointT, GeneratorT>;
    using GeneratorParameters = Class::GeneratorParameters;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<GeneratorParameters>(), "params"_a);
    cls.def(py::init<GeneratorParameters, GeneratorParameters, GeneratorParameters>(), "x_params"_a, "y_params"_a, "z_params"_a);
    cls.def("fill", py::overload_cast<pcl::PointCloud<PointT> &> (&Class::fill), "cloud"_a);
    cls.def("fill", py::overload_cast<int, int, pcl::PointCloud<PointT> &> (&Class::fill), "width"_a, "height"_a, "cloud"_a);
    cls.def("setParameters", py::overload_cast<const GeneratorParameters &> (&Class::setParameters), "params"_a);
    cls.def("setParametersForX", &Class::setParametersForX, "x_params"_a);
    cls.def("setParametersForY", &Class::setParametersForY, "y_params"_a);
    cls.def("setParametersForZ", &Class::setParametersForZ, "z_params"_a);
    cls.def("getParametersForX", &Class::getParametersForX);
    cls.def("getParametersForY", &Class::getParametersForY);
    cls.def("getParametersForZ", &Class::getParametersForZ);
    cls.def("get", &Class::get);
        
}

void defineCommonGenerateFunctions(py::module &m) {
}

void defineCommonGenerateClasses(py::module &sub_module) {
}