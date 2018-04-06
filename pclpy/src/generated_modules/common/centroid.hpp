
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/centroid.h>



template <typename PointT>
void defineCommonCentroidPoint(py::module &m, std::string const & suffix) {
    using Class = CentroidPoint<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("add", &Class::add);
        
}

template<typename PointT, typename Scalar>
void defineCommonNdCentroidFunctor(py::module &m, std::string const & suffix) {
    using Class = NdCentroidFunctor<PointT, Scalar>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointT, Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(), "p"_a, "centroid"_a);
        
}

void defineCommonCentroidClasses(py::module &sub_module) {
}