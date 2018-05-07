
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/common/random.h>

using namespace pcl::common;


template<typename T>
void defineCommonNormalGenerator(py::module &m, std::string const & suffix) {
    using Class = pcl::common::NormalGenerator<T>;
    using EngineType = Class::EngineType;
    using DistributionType = Class::DistributionType;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<T, T, pcl::uint32_t>(), "mean"_a=0, "sigma"_a=1, "seed"_a=-1);
    cls.def(py::init<Parameters>(), "parameters"_a);
    cls.def_readwrite("parameters_", &Class::parameters_);
    cls.def_readwrite("distribution_", &Class::distribution_);
    cls.def_readwrite("rng_", &Class::rng_);
    cls.def_readwrite("generator_", &Class::generator_);
    cls.def("run", &Class::run);
    cls.def("setSeed", &Class::setSeed, "seed"_a);
    cls.def("setParameters", py::overload_cast<T, T, pcl::uint32_t> (&Class::setParameters), "mean"_a, "sigma"_a, "seed"_a=-1);
    cls.def("setParameters", py::overload_cast<const Class::Parameters &> (&Class::setParameters), "parameters"_a);
    cls.def("getParameters", &Class::getParameters);
        
}

template<typename T>
void defineCommonUniformGenerator(py::module &m, std::string const & suffix) {
    using Class = pcl::common::UniformGenerator<T>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<T, T, pcl::uint32_t>(), "min"_a=0, "max"_a=1, "seed"_a=-1);
    cls.def(py::init<Parameters>(), "parameters"_a);
    cls.def("run", &Class::run);
    cls.def("setSeed", &Class::setSeed, "seed"_a);
    cls.def("setParameters", py::overload_cast<T, T, pcl::uint32_t> (&Class::setParameters), "min"_a, "max"_a, "seed"_a=-1);
    cls.def("setParameters", py::overload_cast<const Class::Parameters &> (&Class::setParameters), "parameters"_a);
    cls.def("getParameters", &Class::getParameters);
        
}

template<typename T>
void defineCommonnormal_distribution(py::module &m, std::string const & suffix) {
    using Class = pcl::common::normal_distribution<T>;
    using type = Class::type;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

void defineCommonRandomFunctions(py::module &m) {
}

void defineCommonRandomClasses(py::module &sub_module) {
}