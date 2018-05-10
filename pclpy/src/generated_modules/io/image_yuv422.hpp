
#include <pcl/io/image_yuv422.h>

using namespace pcl::io;


void defineIoImageYUV422(py::module &m) {
    using Class = pcl::io::ImageYUV422;
    py::class_<Class, pcl::io::Image, boost::shared_ptr<Class>> cls(m, "ImageYUV422");
    cls.def(py::init<FrameWrapper::Ptr>(), "image_metadata"_a);
    cls.def(py::init<FrameWrapper::Ptr, Class::Timestamp>(), "image_metadata"_a, "timestamp"_a);
    cls.def("fillRGB", &Class::fillRGB, "width"_a, "height"_a, "rgb_buffer"_a, "rgb_line_step"_a=0);
    cls.def("fillGrayscale", &Class::fillGrayscale, "width"_a, "height"_a, "gray_buffer"_a, "gray_line_step"_a=0);
    cls.def("isResizingSupported", &Class::isResizingSupported, "input_width"_a, "input_height"_a, "output_width"_a, "output_height"_a);
    cls.def("getEncoding", &Class::getEncoding);
}

void defineIoImageYuv422Functions(py::module &m) {
}

void defineIoImageYuv422Classes(py::module &sub_module) {
    defineIoImageYUV422(sub_module);
}