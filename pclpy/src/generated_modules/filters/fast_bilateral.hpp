
#include <pcl/filters/fast_bilateral.h>



template<typename PointT>
void defineFiltersFastBilateralFilter(py::module &m, std::string const & suffix) {
    using Class = pcl::FastBilateralFilter<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("applyFilter", py::overload_cast<pcl::Filter<PointT>::PointCloud &> (&Class::applyFilter), "output"_a);
    cls.def("setSigmaS", &Class::setSigmaS, "sigma_s"_a);
    cls.def("setSigmaR", &Class::setSigmaR, "sigma_r"_a);
    cls.def("getSigmaS", &Class::getSigmaS);
    cls.def("getSigmaR", &Class::getSigmaR);
        
}

void defineFiltersFastBilateralFunctions(py::module &m) {
}

void defineFiltersFastBilateralClasses(py::module &sub_module) {
    py::module sub_module_FastBilateralFilter = sub_module.def_submodule("FastBilateralFilter", "Submodule FastBilateralFilter");
    defineFiltersFastBilateralFilter<pcl::PointXYZ>(sub_module_FastBilateralFilter, "PointXYZ");
    defineFiltersFastBilateralFilter<pcl::PointXYZRGB>(sub_module_FastBilateralFilter, "PointXYZRGB");
    defineFiltersFastBilateralFilter<pcl::PointXYZRGBA>(sub_module_FastBilateralFilter, "PointXYZRGBA");
}