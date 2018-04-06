
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/fast_bilateral_omp.h>



template<typename PointT>
void defineFiltersFastBilateralFilterOMP(py::module &m, std::string const & suffix) {
    using Class = FastBilateralFilterOMP<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FastBilateralFilter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<unsigned int>(), "nr_threads"_a=0);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
    cls.def("apply_filter", py::overload_cast<Filter<PointT>::PointCloud &> (&Class::applyFilter));
        
}

void defineFiltersFastBilateralOmpClasses(py::module &sub_module) {
    py::module sub_module_FastBilateralFilterOMP = sub_module.def_submodule("FastBilateralFilterOMP", "Submodule FastBilateralFilterOMP");
    defineFiltersFastBilateralFilterOMP<PointXYZ>(sub_module_FastBilateralFilterOMP, "PointXYZ");
    defineFiltersFastBilateralFilterOMP<PointXYZRGB>(sub_module_FastBilateralFilterOMP, "PointXYZRGB");
    defineFiltersFastBilateralFilterOMP<PointXYZRGBA>(sub_module_FastBilateralFilterOMP, "PointXYZRGBA");
}