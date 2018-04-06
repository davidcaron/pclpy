#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <pcl/visualization/cloud_viewer.h>

namespace py = pybind11;
using namespace pybind11::literals;


template <typename T>
void showCloud(boost::shared_ptr<pcl::PointCloud<T>> &pc) {
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(pc);
    while (!viewer.wasStopped())
    {
    }
}