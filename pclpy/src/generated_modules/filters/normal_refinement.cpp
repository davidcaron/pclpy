
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/filters/normal_refinement.h>



template<typename NormalT>
void defineFiltersNormalRefinement(py::module &m, std::string const & suffix) {
    using Class = pcl::NormalRefinement<NormalT>;
    py::class_<Class, pcl::Filter<NormalT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<std::vector< std::vector<int> >, std::vector< std::vector<float> >>(), "k_indices"_a, "k_sqr_distances"_a);
    cls.def("setCorrespondences", &Class::setCorrespondences, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("setMaxIterations", &Class::setMaxIterations, "max_iterations"_a);
    cls.def("setConvergenceThreshold", &Class::setConvergenceThreshold, "convergence_threshold"_a);
    cls.def("getCorrespondences", &Class::getCorrespondences, "k_indices"_a, "k_sqr_distances"_a);
    cls.def("getMaxIterations", &Class::getMaxIterations);
    cls.def("getConvergenceThreshold", &Class::getConvergenceThreshold);
        
}

template <typename NormalT>
void defineFiltersNormalRefinementFunctions1(py::module &m) {
    m.def("refineNormal", py::overload_cast<const pcl::PointCloud<NormalT> &, int, const std::vector<int> &, const std::vector<float> &, NormalT &> (&pcl::refineNormal<NormalT>), "cloud"_a, "index"_a, "k_indices"_a, "k_sqr_distances"_a, "point"_a);
}

void defineFiltersNormalRefinementFunctions(py::module &m) {
    defineFiltersNormalRefinementFunctions1<pcl::Normal>(m);
    defineFiltersNormalRefinementFunctions1<pcl::PointNormal>(m);
    defineFiltersNormalRefinementFunctions1<pcl::PointXYZRGBNormal>(m);
    defineFiltersNormalRefinementFunctions1<pcl::PointXYZINormal>(m);
    defineFiltersNormalRefinementFunctions1<pcl::PointXYZLNormal>(m);
    defineFiltersNormalRefinementFunctions1<pcl::PointSurfel>(m);
}

void defineFiltersNormalRefinementClasses(py::module &sub_module) {
    py::module sub_module_NormalRefinement = sub_module.def_submodule("NormalRefinement", "Submodule NormalRefinement");
    defineFiltersNormalRefinement<pcl::Normal>(sub_module_NormalRefinement, "Normal");
    defineFiltersNormalRefinement<pcl::PointNormal>(sub_module_NormalRefinement, "PointNormal");
    defineFiltersNormalRefinement<pcl::PointSurfel>(sub_module_NormalRefinement, "PointSurfel");
    defineFiltersNormalRefinement<pcl::PointXYZINormal>(sub_module_NormalRefinement, "PointXYZINormal");
    defineFiltersNormalRefinement<pcl::PointXYZLNormal>(sub_module_NormalRefinement, "PointXYZLNormal");
    defineFiltersNormalRefinement<pcl::PointXYZRGBNormal>(sub_module_NormalRefinement, "PointXYZRGBNormal");
    defineFiltersNormalRefinementFunctions(sub_module);
}