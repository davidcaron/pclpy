
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/2d/convolution.h>



template <typename PointT>
void define2dConvolution(py::module &m, std::string const & suffix) {
    using Class = Convolution<PointT>;
    py::class_<Class, Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::BOUNDARY_OPTIONS_ENUM>(cls, "boundary_options_enum")
        .value("BOUNDARY_OPTION_CLAMP", Class::BOUNDARY_OPTIONS_ENUM::BOUNDARY_OPTION_CLAMP)
        .value("BOUNDARY_OPTION_MIRROR", Class::BOUNDARY_OPTIONS_ENUM::BOUNDARY_OPTION_MIRROR)
        .value("BOUNDARY_OPTION_ZERO_PADDING", Class::BOUNDARY_OPTIONS_ENUM::BOUNDARY_OPTION_ZERO_PADDING)
        .export_values();
    cls.def(py::init<>());
    cls.def("set_kernel", &Class::setKernel);
    cls.def("set_boundary_options", &Class::setBoundaryOptions);
    cls.def("filter", &Class::filter);
        
}

void define2dPointXYZIEdge(py::module &m) {
    using Class = PointXYZIEdge;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PointXYZIEdge");
    cls.def_readonly("magnitude", &Class::magnitude);
    cls.def_readonly("direction", &Class::direction);
    cls.def_readonly("magnitude_x", &Class::magnitude_x);
    cls.def_readonly("magnitude_y", &Class::magnitude_y);
}

void define2dConvolutionClasses(py::module &sub_module) {
    define2dPointXYZIEdge(sub_module);
}