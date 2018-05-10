
#include <pcl/filters/sampling_surface_normal.h>



template<typename PointT>
void defineFiltersSamplingSurfaceNormal(py::module &m, std::string const & suffix) {
    using Class = pcl::SamplingSurfaceNormal<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Filter<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setSample", &Class::setSample, "sample"_a);
    cls.def("setSeed", &Class::setSeed, "seed"_a);
    cls.def("setRatio", &Class::setRatio, "ratio"_a);
    cls.def("getSample", &Class::getSample);
    cls.def("getSeed", &Class::getSeed);
    cls.def("getRatio", &Class::getRatio);
        
}

void defineFiltersSamplingSurfaceNormalFunctions(py::module &m) {
}

void defineFiltersSamplingSurfaceNormalClasses(py::module &sub_module) {
    py::module sub_module_SamplingSurfaceNormal = sub_module.def_submodule("SamplingSurfaceNormal", "Submodule SamplingSurfaceNormal");
    defineFiltersSamplingSurfaceNormal<pcl::PointNormal>(sub_module_SamplingSurfaceNormal, "PointNormal");
    defineFiltersSamplingSurfaceNormal<pcl::PointXYZINormal>(sub_module_SamplingSurfaceNormal, "PointXYZINormal");
    defineFiltersSamplingSurfaceNormal<pcl::PointXYZRGBNormal>(sub_module_SamplingSurfaceNormal, "PointXYZRGBNormal");
}