
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/grabcut_segmentation.h>



template <typename PointT>
void defineSegmentationGrabCut(py::module &m, std::string const & suffix) {
    using Class = GrabCut<PointT>;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<uint32_t, float>(), "K"_a=5, "lambda"_a=50f);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("set_background_points", &Class::setBackgroundPoints);
    cls.def_property("lambda", &Class::getLambda, &Class::setLambda);
    cls.def_property("k", &Class::getK, &Class::setK);
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("number_of_neighbours", &Class::getNumberOfNeighbours, &Class::setNumberOfNeighbours);
    cls.def("refine", &Class::refine);
    cls.def("refine_once", &Class::refineOnce);
    cls.def("extract", &Class::extract);
    cls.def("set_background_points_indices", py::overload_cast<int, int, int, int> (&Class::setBackgroundPointsIndices));
    cls.def("set_background_points_indices", py::overload_cast<const Class::PointIndicesConstPtr &> (&Class::setBackgroundPointsIndices));
        
}

void defineSegmentationGrabcutSegmentationClasses(py::module &sub_module) {
}