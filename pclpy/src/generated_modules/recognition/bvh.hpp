
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/bvh.h>

using namespace pcl::recognition;


template<class UserData>
void defineRecognitionBVH(py::module &m, std::string const & suffix) {
    using Class = recognition::BVH<UserData>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("build", py::overload_cast<std::vector<BoundedObject *> &> (&Class::build));
    cls.def("clear", &Class::clear);
    cls.def("intersect", &Class::intersect);
        
}

void defineRecognitionBvhClasses(py::module &sub_module) {
}