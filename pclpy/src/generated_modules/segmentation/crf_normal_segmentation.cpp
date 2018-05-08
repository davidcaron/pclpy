
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

#include <pcl/segmentation/crf_normal_segmentation.h>



template <typename PointT>
void defineSegmentationCrfNormalSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::CrfNormalSegmentation<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("segmentPoints", &Class::segmentPoints);
    cls.def("setCloud", &Class::setCloud, "input_cloud"_a);
        
}

void defineSegmentationCrfNormalSegmentationFunctions(py::module &m) {
}

void defineSegmentationCrfNormalSegmentationClasses(py::module &sub_module) {
}