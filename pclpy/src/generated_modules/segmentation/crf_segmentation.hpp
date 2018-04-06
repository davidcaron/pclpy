
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/segmentation/crf_segmentation.h>



template <typename PointT>
void defineSegmentationCrfSegmentation(py::module &m, std::string const & suffix) {
    using Class = CrfSegmentation<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("set_annotated_cloud", &Class::setAnnotatedCloud);
    cls.def("set_normal_cloud", &Class::setNormalCloud);
    cls.def("set_voxel_grid_leaf_size", &Class::setVoxelGridLeafSize);
    cls.def("set_number_of_iterations", &Class::setNumberOfIterations);
    cls.def("set_smoothness_kernel_parameters", &Class::setSmoothnessKernelParameters);
    cls.def("set_appearance_kernel_parameters", &Class::setAppearanceKernelParameters);
    cls.def("set_surface_kernel_parameters", &Class::setSurfaceKernelParameters);
    cls.def("segment_points", &Class::segmentPoints);
    cls.def("create_voxel_grid", &Class::createVoxelGrid);
    cls.def("create_data_vector_from_voxel_grid", &Class::createDataVectorFromVoxelGrid);
    cls.def("create_unary_potentials", &Class::createUnaryPotentials);
        
}

void defineSegmentationCrfSegmentationClasses(py::module &sub_module) {
}