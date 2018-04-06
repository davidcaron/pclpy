
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/clipper3D.h>



template<typename PointT>
void defineFiltersClipper3D(py::module &m, std::string const & suffix) {
    using Class = Clipper3D<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("clip_planar_polygon3_d", py::overload_cast<std::vector<PointT, Eigen::aligned_allocator<PointT> > &> (&Class::clipPlanarPolygon3D, py::const_));
    cls.def("clip_planar_polygon3_d", py::overload_cast<const std::vector<PointT, Eigen::aligned_allocator<PointT> > &, std::vector<PointT, Eigen::aligned_allocator<PointT> > &> (&Class::clipPlanarPolygon3D, py::const_));
    cls.def("clip_point3_d", &Class::clipPoint3D);
    cls.def("clip_line_segment3_d", &Class::clipLineSegment3D);
    cls.def("clip_point_cloud3_d", &Class::clipPointCloud3D);
    cls.def("clone", &Class::clone);
        
}

void defineFiltersClipper3DClasses(py::module &sub_module) {
}