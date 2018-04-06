
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/normal_refinement.h>



template<typename NormalT>
void defineFiltersNormalRefinement(py::module &m, std::string const & suffix) {
    using Class = NormalRefinement<NormalT>;
    py::class_<Class, Filter<NormalT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<std::vector< std::vector<int> >, std::vector< std::vector<float> >>(), "k_indices"_a, "k_sqr_distances"_a);
    cls.def_property("correspondences", &Class::getCorrespondences, &Class::setCorrespondences);
    cls.def_property("max_iterations", &Class::getMaxIterations, &Class::setMaxIterations);
    cls.def_property("convergence_threshold", &Class::getConvergenceThreshold, &Class::setConvergenceThreshold);
        
}

void defineFiltersNormalRefinementClasses(py::module &sub_module) {
    py::module sub_module_NormalRefinement = sub_module.def_submodule("NormalRefinement", "Submodule NormalRefinement");
    defineFiltersNormalRefinement<Normal>(sub_module_NormalRefinement, "Normal");
    defineFiltersNormalRefinement<PointNormal>(sub_module_NormalRefinement, "PointNormal");
    defineFiltersNormalRefinement<PointSurfel>(sub_module_NormalRefinement, "PointSurfel");
    defineFiltersNormalRefinement<PointXYZINormal>(sub_module_NormalRefinement, "PointXYZINormal");
    defineFiltersNormalRefinement<PointXYZLNormal>(sub_module_NormalRefinement, "PointXYZLNormal");
    defineFiltersNormalRefinement<PointXYZRGBNormal>(sub_module_NormalRefinement, "PointXYZRGBNormal");
}