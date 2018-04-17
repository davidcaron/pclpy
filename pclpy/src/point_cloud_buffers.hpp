#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


template<typename BufferT, typename PointT, ssize_t OFFSET, uint8_t SIZE>
py::array buffer(boost::shared_ptr<PointCloud<PointT>> &cloud)
{
    std::vector<ssize_t> shape {static_cast<ssize_t> (cloud->size())};
    std::vector<ssize_t> strides {sizeof(PointT)};
    int ndim = 1;
    if (SIZE > 1) {
        shape.push_back(SIZE);
        strides.push_back(sizeof(BufferT));
        ndim = 2;
    }
    py::buffer_info buf = py::buffer_info(
        (BufferT *) (cloud->points.data()) + OFFSET,    /* Pointer to buffer */
        sizeof(BufferT),                                    /* Size of one scalar */
        py::format_descriptor<BufferT>::format(),           /* Python struct-style format descriptor */
        ndim,                                                /* Number of dimensions */
        shape,                                            /* Shape */
        strides              /* Strides (in bytes) for each index */
    );
    return py::array(buf);
}

template<typename T>
void defineBuffers(py::class_<PointCloud<T>, boost::shared_ptr<PointCloud<T>>> &cls) {
};

template<>
void defineBuffers<PointXYZ>(py::class_<PointCloud<PointXYZ>, boost::shared_ptr<PointCloud<PointXYZ>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZ, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZ, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZ, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZ, 2, 1>);
}

template<>
void defineBuffers<PointXYZI>(py::class_<PointCloud<PointXYZI>, boost::shared_ptr<PointCloud<PointXYZI>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZI, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZI, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZI, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZI, 2, 1>);
    cls.def_property_readonly("intensity", &buffer<float, PointXYZI, 4, 1>);
}

template<>
void defineBuffers<PointXYZL>(py::class_<PointCloud<PointXYZL>, boost::shared_ptr<PointCloud<PointXYZL>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZL, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZL, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZL, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZL, 2, 1>);
    cls.def_property_readonly("label", &buffer<uint32_t, PointXYZL, 4, 1>);
}

template<>
void defineBuffers<Label>(py::class_<PointCloud<Label>, boost::shared_ptr<PointCloud<Label>>> &cls) {
    cls.def_property_readonly("label", &buffer<uint32_t, Label, 0, 1>);
}

template<>
void defineBuffers<PointXYZRGBA>(py::class_<PointCloud<PointXYZRGBA>, boost::shared_ptr<PointCloud<PointXYZRGBA>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZRGBA, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZRGBA, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZRGBA, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZRGBA, 2, 1>);
    cls.def_property_readonly("rgb_reversed", &buffer<uint8_t, PointXYZRGBA, 16, 3>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointXYZRGBA, 16, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointXYZRGBA, 17, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointXYZRGBA, 18, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointXYZRGBA, 19, 1>);
}

template<>
void defineBuffers<PointXYZRGB>(py::class_<PointCloud<PointXYZRGB>, boost::shared_ptr<PointCloud<PointXYZRGB>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZRGB, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZRGB, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZRGB, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZRGB, 2, 1>);
    cls.def_property_readonly("rgb_reversed", &buffer<uint8_t, PointXYZRGB, 16, 3>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointXYZRGB, 16, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointXYZRGB, 17, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointXYZRGB, 18, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointXYZRGB, 19, 1>);
}

template<>
void defineBuffers<PointXYZRGBL>(py::class_<PointCloud<PointXYZRGBL>, boost::shared_ptr<PointCloud<PointXYZRGBL>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZRGBL, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZRGBL, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZRGBL, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZRGBL, 2, 1>);
    cls.def_property_readonly("rgb_reversed", &buffer<uint8_t, PointXYZRGBL, 16, 3>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointXYZRGBL, 16, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointXYZRGBL, 17, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointXYZRGBL, 18, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointXYZRGBL, 19, 1>);
    cls.def_property_readonly("label", &buffer<uint32_t, PointXYZRGBL, 5, 1>);
}

template<>
void defineBuffers<PointXYZHSV>(py::class_<PointCloud<PointXYZHSV>, boost::shared_ptr<PointCloud<PointXYZHSV>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZHSV, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZHSV, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZHSV, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZHSV, 2, 1>);
    cls.def_property_readonly("hsv", &buffer<float, PointXYZHSV, 4, 3>);
    cls.def_property_readonly("h", &buffer<float, PointXYZHSV, 4, 1>);
    cls.def_property_readonly("s", &buffer<float, PointXYZHSV, 5, 1>);
    cls.def_property_readonly("v", &buffer<float, PointXYZHSV, 6, 1>);
}

template<>
void defineBuffers<PointNormal>(py::class_<PointCloud<PointNormal>, boost::shared_ptr<PointCloud<PointNormal>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointNormal, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointNormal, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointNormal, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointNormal, 2, 1>);
    cls.def_property_readonly("normals", &buffer<float, PointNormal, 4, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, PointNormal, 4, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, PointNormal, 5, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, PointNormal, 6, 1>);
    cls.def_property_readonly("curvature", &buffer<float, PointNormal, 8, 1>);
}

template<>
void defineBuffers<Normal>(py::class_<PointCloud<Normal>, boost::shared_ptr<PointCloud<Normal>>> &cls) {
    cls.def_property_readonly("normals", &buffer<float, Normal, 0, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, Normal, 0, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, Normal, 1, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, Normal, 2, 1>);
    cls.def_property_readonly("curvature", &buffer<float, Normal, 4, 1>);
}
