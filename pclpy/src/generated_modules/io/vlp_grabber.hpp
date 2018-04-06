
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/vlp_grabber.h>



void defineIoVLPGrabber(py::module &m) {
    using Class = VLPGrabber;
    py::class_<Class, HDLGrabber, boost::shared_ptr<Class>> cls(m, "VLPGrabber");
}

void defineIoVlpGrabberClasses(py::module &sub_module) {
    defineIoVLPGrabber(sub_module);
}