
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/integral_image2D.h>



template <class DataType, unsigned Dimension>
void defineFeaturesIntegralImage2D(py::module &m, std::string const & suffix) {
    using Class = IntegralImage2D<DataType, Dimension>;
    using ElementType = Class::ElementType;
    using SecondOrderType = Class::SecondOrderType;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "compute_second_order_integral_images"_a);
    cls.def("set_second_order_computation", &Class::setSecondOrderComputation);
    cls.def("set_input", &Class::setInput);
    cls.def_readonly_static("second_order_size", &Class::second_order_size);
        
}



template <typename DataType>
void defineFeaturesIntegralImageTypeTraits(py::module &m, std::string const & suffix) {
    using Class = IntegralImageTypeTraits<DataType>;
    using Type = Class::Type;
    using IntegralType = Class::IntegralType;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

void defineFeaturesIntegralImage2DClasses(py::module &sub_module) {
}