
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/point_cloud.h>



template <typename PointOutT>
void defineNdCopyEigenPointFunctor(py::module &m, std::string const & suffix) {
    using Class = NdCopyEigenPointFunctor<PointOutT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<Eigen::VectorXf, PointOutT>(), "p1"_a, "p2"_a);
        
}

template <typename PointInT>
void defineNdCopyPointEigenFunctor(py::module &m, std::string const & suffix) {
    using Class = NdCopyPointEigenFunctor<PointInT>;
    using Pod = Class::Pod;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointInT, Eigen::VectorXf>(), "p1"_a, "p2"_a);
        
}

template <typename PointT>
void definePointCloud(py::module &m, std::string const & suffix) {
    using Class = PointCloud<PointT>;
    using PointType = Class::PointType;
    using VectorType = Class::VectorType;
    using CloudVectorType = Class::CloudVectorType;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using value_type = Class::value_type;
    using reference = Class::reference;
    using const_reference = Class::const_reference;
    using difference_type = Class::difference_type;
    using size_type = Class::size_type;
    using iterator = Class::iterator;
    using const_iterator = Class::const_iterator;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<PointCloud<PointT>, std::vector<int>>(), "pc"_a, "indices"_a);
    cls.def(py::init<uint32_t, uint32_t, PointT>(), "width_"_a, "height_"_a, "value_"_a=PointT());
    cls.def_readonly("header", &Class::header);
    cls.def_readonly("points", &Class::points);
    cls.def_readonly("width", &Class::width);
    cls.def_readonly("height", &Class::height);
    cls.def_readonly("is_dense", &Class::is_dense);
    cls.def_readonly("sensor_origin_", &Class::sensor_origin_);
    cls.def_readonly("sensor_orientation_", &Class::sensor_orientation_);
    cls.def("at", py::overload_cast<int, int> (&Class::at, py::const_));
    cls.def("at", py::overload_cast<int, int> (&Class::at));
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("begin", py::overload_cast<> (&Class::begin));
    cls.def("end", py::overload_cast<> (&Class::end));
    cls.def("begin", py::overload_cast<> (&Class::begin, py::const_));
    cls.def("end", py::overload_cast<> (&Class::end, py::const_));
    // Operators not implemented (operator[]);
    // Operators not implemented (operator[]);
    cls.def("at", py::overload_cast<size_t> (&Class::at, py::const_));
    cls.def("at", py::overload_cast<size_t> (&Class::at));
    cls.def("front", py::overload_cast<> (&Class::front, py::const_));
    cls.def("front", py::overload_cast<> (&Class::front));
    cls.def("back", py::overload_cast<> (&Class::back, py::const_));
    cls.def("back", py::overload_cast<> (&Class::back));
    cls.def("insert", py::overload_cast<iterator, const PointT &> (&Class::insert));
    cls.def("insert", py::overload_cast<iterator, size_t, const PointT &> (&Class::insert));
    cls.def("erase", py::overload_cast<iterator> (&Class::erase));
    cls.def("erase", py::overload_cast<iterator, iterator> (&Class::erase));
    // Operators not implemented (operator+=);
    // Operators not implemented (operator+);
    cls.def("is_organized", &Class::isOrganized);
    cls.def("size", &Class::size);
    cls.def("reserve", &Class::reserve);
    cls.def("empty", &Class::empty);
    cls.def("resize", &Class::resize);
    cls.def("push_back", &Class::push_back);
    cls.def("swap", &Class::swap);
    cls.def("clear", &Class::clear);
    cls.def("make_shared", &Class::makeShared);
    cls.def("get_matrix_xf_map", py::overload_cast<int, int, int> (&Class::getMatrixXfMap));
    cls.def("get_matrix_xf_map", py::overload_cast<int, int, int> (&Class::getMatrixXfMap, py::const_));
    cls.def("get_matrix_xf_map", py::overload_cast<> (&Class::getMatrixXfMap));
    cls.def("get_matrix_xf_map", py::overload_cast<> (&Class::getMatrixXfMap, py::const_));
        
}

void definePointCloudClasses(py::module &sub_module) {
}