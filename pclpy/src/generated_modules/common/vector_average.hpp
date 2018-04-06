
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/vector_average.h>



template <typename real, int dimension>
void defineCommonVectorAverage(py::module &m, std::string const & suffix) {
    using Class = VectorAverage<real>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("do_pca", py::overload_cast<Eigen::Matrix<real, dimension, 1> &, Eigen::Matrix<real, dimension, 1> &, Eigen::Matrix<real, dimension, 1> &, Eigen::Matrix<real, dimension, 1> &> (&Class::doPCA, py::const_));
    cls.def("do_pca", py::overload_cast<Eigen::Matrix<real, dimension, 1> &> (&Class::doPCA, py::const_));
    cls.def("reset", &Class::reset);
    cls.def("add", &Class::add);
        
}

void defineCommonVectorAverageClasses(py::module &sub_module) {
}