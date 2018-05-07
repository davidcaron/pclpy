
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/search/flann_search.h>

using namespace pcl::search;


template<typename PointT, typename FlannDistance=flann::L2_Simple <float> >
void defineSearchFlannSearch(py::module &m, std::string const & suffix) {
    using Class = pcl::search::FlannSearch<PointT, FlannDistance=flann::L2_Simple>;
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
    py::class_<Class, pcl::search::Search<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool, FlannIndexCreatorPtr>(), "sorted"_a=true, "creator"_a=FlannIndexCreatorPtr(newKdTreeIndexCreator()));
    cls.def("nearestKSearch", py::overload_cast<const PointT &, int, std::vector<int> &, std::vector<float> &> (&Class::nearestKSearch, py::const_), "point"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("nearestKSearch", py::overload_cast<const PointCloud &, const std::vector<int> &, int, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &> (&Class::nearestKSearch, py::const_), "cloud"_a, "indices"_a, "k"_a, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("radiusSearch", py::overload_cast<const PointT &, double, std::vector<int> &, std::vector<float> &, unsigned int> (&Class::radiusSearch, py::const_), "point"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("radiusSearch", py::overload_cast<const PointCloud &, const std::vector<int> &, double, std::vector< std::vector<int> > &, std::vector< std::vector<float> > &, unsigned int> (&Class::radiusSearch, py::const_), "cloud"_a, "indices"_a, "radius"_a, "k_indices"_a, "k_sqr_distances"_a, "max_nn"_a=0);
    cls.def("setEpsilon", &Class::setEpsilon, "eps"_a);
    cls.def("setChecks", &Class::setChecks, "checks"_a);
    cls.def("setInputCloud", py::overload_cast<const PointCloudConstPtr &, const IndicesConstPtr &> (&Class::setInputCloud), "cloud"_a, "indices"_a=pcl::IndicesConstPtr());
    cls.def("setPointRepresentation", &Class::setPointRepresentation, "point_representation"_a);
    cls.def("getEpsilon", &Class::getEpsilon);
    cls.def("getChecks", &Class::getChecks);
    cls.def("getPointRepresentation", &Class::getPointRepresentation);
        
}

void defineSearchFlannSearchFunctions(py::module &m) {
}

void defineSearchFlannSearchClasses(py::module &sub_module) {
}