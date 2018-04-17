
#pragma warning (disable : 4367)
#pragma warning (disable : 4267)

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <pcl/point_representation.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);

#include "make_opaque_vectors.hpp"  // must be first for PYBIND11_MAKE_OPAQUE to work
#include "eigen_bind.hpp"
#include "generated_modules/__main_loader.hpp"
#include "point_cloud_buffers.hpp"
#include "point_cloud_from_array.hpp"
#include "vector_classes.hpp"
#include "point_types.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

//template <typename T>
//static py::class_<PointCloud<T>, boost::shared_ptr<PointCloud<T>>>
//                    definePointCloud(py::module & m, std::string const & suffix) {
//    using Class = PointCloud<T>;
//    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
//    cls.def(py::init<>());
//    cls.def("size", &Class::size);
//    cls.def_readwrite("points", &Class::points);
//    return cls;
//}

template <typename T>
void definePointCloudBuffers(py::module &m, const char* suffix) {
    using PointCloud = pcl::PointCloud<T>;
    using Type = py::class_<PointCloud, boost::shared_ptr<PointCloud>>;
    auto pc = static_cast<Type>(m.attr(suffix));
    pc.def_static("from_array", &fromArray<T>);
    defineBuffers<T>(pc);
}

template <typename T>
void definePointCloudBuffersRGB(py::module &m, const char* suffix) {
    using PointCloud = pcl::PointCloud<T>;
    using Type = py::class_<PointCloud, boost::shared_ptr<PointCloud>>;
    auto pc = static_cast<Type>(m.attr(suffix));
    pc.def_static("from_array", &fromArrayRGB<T>);
    defineBuffers<T>(pc);
}

PYBIND11_MODULE(pcl, m) {
    m.doc() = "PCL python bindings";

    defineEigenClasses(m);
    definePointTypes(m);
    defineVectorClasses(m);

    defineClasses(m);

    py::module pc = m.attr("PointCloud");
    definePointCloudBuffers<PointXYZ>(pc, "PointXYZ");
    definePointCloudBuffers<PointXYZI>(pc, "PointXYZI");
    definePointCloudBuffers<PointXYZL>(pc, "PointXYZL");
    definePointCloudBuffersRGB<PointXYZRGBA>(pc, "PointXYZRGBA");
    definePointCloudBuffers<PointNormal>(pc, "PointNormal");
    definePointCloudBuffers<Normal>(pc, "Normal");

//    auto xyz = static_cast<py::class_<pcl::PointCloud<pcl::PointXYZ>,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>>(pc.attr("PointXYZ"));
//    xyz.def_static("from_array", &fromArray<PointXYZ>);
//    defineBuffers<PointXYZ>(xyz);

//    auto xyzl = definePointCloud<PointXYZL>(m_point_cloud, "PointXYZL");
//    xyzl.def_static("from_array", &fromArray<PointXYZL>);
//    defineBuffers<PointXYZL>(xyzl);

//    auto xyzi = definePointCloud<PointXYZI>(m_point_cloud, "PointXYZI");
//    xyzi.def_static("from_array", &fromArray<PointXYZI>);
//    defineBuffers<PointXYZI>(xyzi);

//    auto xyzrgba = definePointCloud<PointXYZRGBA>(m_point_cloud, "PointXYZRGBA");
//    xyzrgba.def_static("from_array", &fromArrayRGB<PointXYZRGBA>);
//    defineBuffers<PointXYZRGBA>(xyzrgba);

//    auto pointNormal = definePointCloud<PointNormal>(m_point_cloud, "PointNormal");
//    pointNormal.def_static("from_array", &fromArray<PointNormal>);
//    defineBuffers<PointNormal>(pointNormal);

//    auto normal = definePointCloud<Normal>(m_point_cloud, "Normal");
//    normal.def_static("from_array", &fromArray<Normal>);
//    defineBuffers<Normal>(normal);

}
