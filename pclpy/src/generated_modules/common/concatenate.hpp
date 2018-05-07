
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/common/concatenate.h>



template<typename PointInT, typename PointOutT>
void defineCommonNdConcatenateFunctor(py::module &m, std::string const & suffix) {
    using Class = pcl::NdConcatenateFunctor<PointInT, PointOutT>;
    using PodIn = Class::PodIn;
    using PodOut = Class::PodOut;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointInT, PointOutT>(), "p1"_a, "p2"_a);
    // Operators not implemented (operator());
        
}

void defineCommonConcatenateFunctions(py::module &m) {
}

void defineCommonConcatenateClasses(py::module &sub_module) {
}