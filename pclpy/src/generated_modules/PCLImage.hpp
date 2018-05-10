
#include <pcl/PCLImage.h>



void definePCLImage(py::module &m) {
    using Class = pcl::PCLImage;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PCLImage");
    cls.def(py::init<>());
    cls.def_readwrite("header", &Class::header);
    cls.def_readwrite("height", &Class::height);
    cls.def_readwrite("width", &Class::width);
    cls.def_readwrite("encoding", &Class::encoding);
    cls.def_readwrite("is_bigendian", &Class::is_bigendian);
    cls.def_readwrite("step", &Class::step);
    cls.def_readwrite("data", &Class::data);
}

void definePCLImageFunctions(py::module &m) {
}

void definePCLImageClasses(py::module &sub_module) {
    definePCLImage(sub_module);
    definePCLImageFunctions(sub_module);
}