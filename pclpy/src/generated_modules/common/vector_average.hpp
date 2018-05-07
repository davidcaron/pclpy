
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/common/vector_average.h>



template <typename real, int dimension>
void defineCommonVectorAverage(py::module &m, std::string const & suffix) {
    using Class = pcl::VectorAverage<real>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("reset", &Class::reset);
    cls.def("add", &Class::add, "sample"_a, "weight"_a=1.0);
    cls.def("doPCA", py::overload_cast<Eigen::Matrix<real, dimension, 1> &, Eigen::Matrix<real, dimension, 1> &, Eigen::Matrix<real, dimension, 1> &, Eigen::Matrix<real, dimension, 1> &> (&Class::doPCA, py::const_), "eigen_values"_a, "eigen_vector1"_a, "eigen_vector2"_a, "eigen_vector3"_a);
    cls.def("doPCA", py::overload_cast<Eigen::Matrix<real, dimension, 1> &> (&Class::doPCA, py::const_), "eigen_values"_a);
    cls.def("getMean", &Class::getMean);
    cls.def("getCovariance", &Class::getCovariance);
    cls.def("getAccumulatedWeight", &Class::getAccumulatedWeight);
    cls.def("getNoOfSamples", &Class::getNoOfSamples);
    cls.def("getEigenVector1", &Class::getEigenVector1, "eigen_vector1"_a);
        
}

void defineCommonVectorAverageFunctions(py::module &m) {
}

void defineCommonVectorAverageClasses(py::module &sub_module) {
}