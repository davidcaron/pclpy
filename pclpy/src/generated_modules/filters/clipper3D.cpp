
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

#include <pcl/filters/clipper3D.h>



template<typename PointT>
void defineFiltersClipper3D(py::module &m, std::string const & suffix) {
    using Class = pcl::Clipper3D<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("clipPoint3D", &Class::clipPoint3D, "point"_a);
    cls.def("clipLineSegment3D", &Class::clipLineSegment3D, "pt1"_a, "pt2"_a);
    cls.def("clipPlanarPolygon3D", py::overload_cast<std::vector<PointT, Eigen::aligned_allocator<PointT> > &> (&Class::clipPlanarPolygon3D, py::const_), "polygon"_a);
    cls.def("clipPlanarPolygon3D", py::overload_cast<const std::vector<PointT, Eigen::aligned_allocator<PointT> > &, std::vector<PointT, Eigen::aligned_allocator<PointT> > &> (&Class::clipPlanarPolygon3D, py::const_), "polygon"_a, "clipped_polygon"_a);
    cls.def("clipPointCloud3D", &Class::clipPointCloud3D, "cloud_in"_a, "clipped"_a, "indices"_a=std::vector<int>());
    cls.def("clone", &Class::clone);
        
}

void defineFiltersClipper3DFunctions(py::module &m) {
}

void defineFiltersClipper3DClasses(py::module &sub_module) {
}