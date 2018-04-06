
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/search/flann_search.h>

using namespace pcl::search;


template<typename PointT, typename FlannDistance=flann::L2_Simple <float> >
void defineSearchFlannSearch(py::module &m, std::string const & suffix) {
    using Class = search::FlannSearch<PointT, FlannDistance=flann::L2_Simple>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using IndicesPtr = Class::IndicesPtr;
    using IndicesConstPtr = Class::IndicesConstPtr;
    using MatrixPtr = Class::MatrixPtr;
    using MatrixConstPtr = Class::MatrixConstPtr;
    using Index = Class::Index;
    using IndexPtr = Class::IndexPtr;
    using PointRepresentation = Class::PointRepresentation;
    using PointRepresentationPtr = Class::PointRepresentationPtr;
    using PointRepresentationConstPtr = Class::PointRepresentationConstPtr;
    using FlannIndexCreatorPtr = Class::FlannIndexCreatorPtr;
    py::class_<Class, Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool, Class::FlannIndexCreatorPtr>(), "sorted"_a=true, "creator"_a=FlannIndexCreatorPtr(newKdTreeIndexCreator()));
    cls.def_property("epsilon", &Class::getEpsilon, &Class::setEpsilon);
    cls.def_property("checks", &Class::getChecks, &Class::setChecks);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def_property("point_representation", &Class::getPointRepresentation, &Class::setPointRepresentation);
    cls.def("nearest_k_search", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_));
    cls.def("nearest_k_search", py::overload_cast<const PointCloud &, const std::vector<int> &, int, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &> (&Class::nearestKSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_));
    cls.def("radius_search", py::overload_cast<const PointCloud &, const std::vector<int> &, double, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &, unsigned int> (&Class::radiusSearch, py::const_));
        
}

void defineSearchFlannSearchClasses(py::module &sub_module) {
}