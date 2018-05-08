
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

#include <pcl/features/integral_image2D.h>



template <class DataType, unsigned Dimension>
void defineFeaturesIntegralImage2D(py::module &m, std::string const & suffix) {
    using Class = pcl::IntegralImage2D<DataType, Dimension>;
    using ElementType = Class::ElementType;
    using SecondOrderType = Class::SecondOrderType;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "compute_second_order_integral_images"_a);
    cls.def_readonly_static("second_order_size", &Class::second_order_size);
    cls.def("setSecondOrderComputation", &Class::setSecondOrderComputation, "compute_second_order_integral_images"_a);
    cls.def("setInput", &Class::setInput, "data"_a, "width"_a, "height"_a, "element_stride"_a, "row_stride"_a);
    cls.def("getFirstOrderSum", &Class::getFirstOrderSum, "start_x"_a, "start_y"_a, "width"_a, "height"_a);
    cls.def("getFirstOrderSumSE", &Class::getFirstOrderSumSE, "start_x"_a, "start_y"_a, "end_x"_a, "end_y"_a);
    cls.def("getSecondOrderSum", &Class::getSecondOrderSum, "start_x"_a, "start_y"_a, "width"_a, "height"_a);
    cls.def("getSecondOrderSumSE", &Class::getSecondOrderSumSE, "start_x"_a, "start_y"_a, "end_x"_a, "end_y"_a);
    cls.def("getFiniteElementsCount", &Class::getFiniteElementsCount, "start_x"_a, "start_y"_a, "width"_a, "height"_a);
    cls.def("getFiniteElementsCountSE", &Class::getFiniteElementsCountSE, "start_x"_a, "start_y"_a, "end_x"_a, "end_y"_a);
        
}

template <typename DataType>
void defineFeaturesIntegralImageTypeTraits(py::module &m, std::string const & suffix) {
    using Class = pcl::IntegralImageTypeTraits<DataType>;
    using Type = Class::Type;
    using IntegralType = Class::IntegralType;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

void defineFeaturesIntegralImage2DFunctions(py::module &m) {
}

void defineFeaturesIntegralImage2DClasses(py::module &sub_module) {
}