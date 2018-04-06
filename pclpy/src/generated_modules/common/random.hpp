
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/random.h>

using namespace pcl::common;


template<typename T>
void defineCommonNormalGenerator(py::module &m, std::string const & suffix) {
    using Class = common::NormalGenerator<T>;
    using EngineType = Class::EngineType;
    using DistributionType = Class::DistributionType;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<T, T, pcl::uint32_t>(), "mean"_a=0, "sigma"_a=1, "seed"_a=-1);
    cls.def(py::init<Parameters>(), "parameters"_a);
    cls.def("set_seed", &Class::setSeed);
    cls.def_readonly("parameters_", &Class::parameters_);
    cls.def_readonly("distribution_", &Class::distribution_);
    cls.def_readonly("rng_", &Class::rng_);
    cls.def_readonly("generator_", &Class::generator_);
    cls.def("run", &Class::run);
    cls.def("set_parameters", py::overload_cast<T, T, pcl::uint32_t> (&Class::setParameters));
    cls.def("set_parameters", py::overload_cast<const Class::Parameters &> (&Class::setParameters));
        
}

template<typename T>
void defineCommonUniformGenerator(py::module &m, std::string const & suffix) {
    using Class = common::UniformGenerator<T>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<T, T, pcl::uint32_t>(), "min"_a=0, "max"_a=1, "seed"_a=-1);
    cls.def(py::init<Parameters>(), "parameters"_a);
    cls.def("set_seed", &Class::setSeed);
    cls.def("run", &Class::run);
    cls.def("set_parameters", py::overload_cast<T, T, pcl::uint32_t> (&Class::setParameters));
    cls.def("set_parameters", py::overload_cast<const Class::Parameters &> (&Class::setParameters));
        
}

template<typename T>
void defineCommonnormal_distribution(py::module &m, std::string const & suffix) {
    using Class = common::normal_distribution<T>;
    using type = Class::type;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}





void defineCommonRandomClasses(py::module &sub_module) {
}