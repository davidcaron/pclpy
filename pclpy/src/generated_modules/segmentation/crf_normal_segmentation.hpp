
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/crf_normal_segmentation.h>



template <typename PointT>
void defineSegmentationCrfNormalSegmentation(py::module &m, std::string const & suffix) {
    using Class = CrfNormalSegmentation<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_cloud", &Class::setCloud);
    cls.def("segment_points", &Class::segmentPoints);
        
}

void defineSegmentationCrfNormalSegmentationClasses(py::module &sub_module) {
}