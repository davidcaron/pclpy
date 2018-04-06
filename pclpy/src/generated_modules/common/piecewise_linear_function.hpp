
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/piecewise_linear_function.h>



void defineCommonPiecewiseLinearFunction(py::module &m) {
    using Class = PiecewiseLinearFunction;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PiecewiseLinearFunction");
    cls.def(py::init<float, float>(), "factor"_a, "offset"_a);
}

void defineCommonPiecewiseLinearFunctionClasses(py::module &sub_module) {
    defineCommonPiecewiseLinearFunction(sub_module);
}