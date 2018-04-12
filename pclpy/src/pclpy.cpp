
#pragma warning (disable : 4367)
#pragma warning (disable : 4267)

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/StdVector>

#include <iostream>
#include <pcl/point_representation.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);

#include "make_opaque_vectors.hpp"  // must be first for PYBIND11_MAKE_OPAQUE to work
#include "generated_modules/__main_loader.hpp"
#include "point_cloud_buffers.hpp"
#include "point_cloud_from_array.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

template <typename T>
static py::class_<PointCloud<T>, boost::shared_ptr<PointCloud<T>>>
                    definePointCloud(py::module & m, std::string const & suffix) {
    using Class = PointCloud<T>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, ("PointCloud" + suffix).c_str());
    cls.def(py::init<>());
    cls.def("size", &Class::size);
    cls.def_readwrite("points", &Class::points);
    return cls;
}

void defineVectorClasses(py::module &m) {
    py::module m_vector = m.def_submodule("vector", "Submodule for vectors");
    py::bind_vector<std::vector<PointIndices>>(m_vector, "PointIndices");
    py::bind_vector<std::vector<int>>(m_vector, "Int", py::buffer_protocol());
    py::bind_vector<std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>>>(m_vector, "PointXYZRGBA");
}

PYBIND11_MODULE(pcl, m) {
    m.doc() = "PCL python bindings";

    defineClasses(m);

    defineVectorClasses(m);

    // PointXYZ
    auto xyz = definePointCloud<PointXYZ>(m, "XYZ");
    xyz.def_static("from_array", &fromArray<PointXYZ>);
    defineBuffers<PointXYZ>(xyz);

    // PointXYZL
    auto xyzl = definePointCloud<PointXYZL>(m, "XYZL");
    xyzl.def_static("from_array", &fromArray<PointXYZL>);
    defineBuffers<PointXYZL>(xyzl);

    // PointXYZI
    auto xyzi = definePointCloud<PointXYZI>(m, "XYZI");
    xyzi.def_static("from_array", &fromArray<PointXYZI>);
    defineBuffers<PointXYZI>(xyzi);

    // PointXYZRGBA
    auto xyzrgba = definePointCloud<PointXYZRGBA>(m, "XYZRGBA");
    xyzrgba.def_static("from_array", &fromArrayRGB<PointXYZRGBA>);
    defineBuffers<PointXYZRGBA>(xyzrgba);

    // PointNormal
    auto pointNormal = definePointCloud<PointNormal>(m, "PointNormal");
    pointNormal.def_static("from_array", &fromArray<PointNormal>);
    defineBuffers<PointNormal>(pointNormal);

    // Normal
    auto normal = definePointCloud<Normal>(m, "Normal");
    normal.def_static("from_array", &fromArray<Normal>);
    defineBuffers<Normal>(normal);

}
