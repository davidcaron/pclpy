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
void defineBuffers<PointXY>(py::class_<PointCloud<PointXY>, boost::shared_ptr<PointCloud<PointXY>>> &cls) {
    cls.def_property_readonly("x", &buffer<float, PointXY, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXY, 1, 1>);
}

template<>
void defineBuffers<InterestPoint>(py::class_<PointCloud<InterestPoint>, boost::shared_ptr<PointCloud<InterestPoint>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, InterestPoint, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, InterestPoint, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, InterestPoint, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, InterestPoint, 2, 1>);
    cls.def_property_readonly("strength", &buffer<float, InterestPoint, 4, 1>);
}

template<>
void defineBuffers<Axis>(py::class_<PointCloud<Axis>, boost::shared_ptr<PointCloud<Axis>>> &cls) {
    cls.def_property_readonly("normals", &buffer<float, Axis, 0, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, Axis, 0, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, Axis, 1, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, Axis, 2, 1>);
}

template<>
void defineBuffers<Normal>(py::class_<PointCloud<Normal>, boost::shared_ptr<PointCloud<Normal>>> &cls) {
    cls.def_property_readonly("normals", &buffer<float, Normal, 0, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, Normal, 0, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, Normal, 1, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, Normal, 2, 1>);
    cls.def_property_readonly("curvature", &buffer<float, Normal, 4, 1>);
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
void defineBuffers<PointXYZRGBNormal>(py::class_<PointCloud<PointXYZRGBNormal>, boost::shared_ptr<PointCloud<PointXYZRGBNormal>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZRGBNormal, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZRGBNormal, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZRGBNormal, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZRGBNormal, 2, 1>);
    cls.def_property_readonly("normals", &buffer<float, PointXYZRGBNormal, 4, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, PointXYZRGBNormal, 4, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, PointXYZRGBNormal, 5, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, PointXYZRGBNormal, 6, 1>);
    cls.def_property_readonly("rgb_reversed", &buffer<uint8_t, PointXYZRGBNormal, 32, 3>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointXYZRGBNormal, 32, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointXYZRGBNormal, 33, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointXYZRGBNormal, 34, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointXYZRGBNormal, 35, 1>);
    cls.def_property_readonly("curvature", &buffer<float, PointXYZRGBNormal, 9, 1>);
}

template<>
void defineBuffers<PointXYZINormal>(py::class_<PointCloud<PointXYZINormal>, boost::shared_ptr<PointCloud<PointXYZINormal>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZINormal, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZINormal, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZINormal, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZINormal, 2, 1>);
    cls.def_property_readonly("normals", &buffer<float, PointXYZINormal, 4, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, PointXYZINormal, 4, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, PointXYZINormal, 5, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, PointXYZINormal, 6, 1>);
    cls.def_property_readonly("intensity", &buffer<float, PointXYZINormal, 8, 1>);
    cls.def_property_readonly("curvature", &buffer<float, PointXYZINormal, 9, 1>);
}

template<>
void defineBuffers<PointXYZLNormal>(py::class_<PointCloud<PointXYZLNormal>, boost::shared_ptr<PointCloud<PointXYZLNormal>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointXYZLNormal, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZLNormal, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZLNormal, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZLNormal, 2, 1>);
    cls.def_property_readonly("normals", &buffer<float, PointXYZLNormal, 4, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, PointXYZLNormal, 4, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, PointXYZLNormal, 5, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, PointXYZLNormal, 6, 1>);
    cls.def_property_readonly("label", &buffer<uint32_t, PointXYZLNormal, 8, 1>);
    cls.def_property_readonly("curvature", &buffer<float, PointXYZLNormal, 9, 1>);
}

template<>
void defineBuffers<PointWithRange>(py::class_<PointCloud<PointWithRange>, boost::shared_ptr<PointCloud<PointWithRange>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointWithRange, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointWithRange, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointWithRange, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointWithRange, 2, 1>);
    cls.def_property_readonly("range", &buffer<float, PointWithRange, 4, 1>);
}

template<>
void defineBuffers<PointWithViewpoint>(py::class_<PointCloud<PointWithViewpoint>, boost::shared_ptr<PointCloud<PointWithViewpoint>>> &cls) {
    cls.def_property_readonly("xyz", &buffer<float, PointWithViewpoint, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointWithViewpoint, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointWithViewpoint, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointWithViewpoint, 2, 1>);
    cls.def_property_readonly("vp", &buffer<float, PointWithViewpoint, 4, 3>);
    cls.def_property_readonly("vp_x", &buffer<float, PointWithViewpoint, 4, 1>);
    cls.def_property_readonly("vp_y", &buffer<float, PointWithViewpoint, 5, 1>);
    cls.def_property_readonly("vp_z", &buffer<float, PointWithViewpoint, 6, 1>);
}

template<>
void defineBuffers<MomentInvariants>(py::class_<PointCloud<MomentInvariants>, boost::shared_ptr<PointCloud<MomentInvariants>>> &cls) {
    cls.def_property_readonly("j1", &buffer<float, MomentInvariants, 0, 1>);
    cls.def_property_readonly("j2", &buffer<float, MomentInvariants, 1, 1>);
    cls.def_property_readonly("j3", &buffer<float, MomentInvariants, 2, 1>);
}

template<>
void defineBuffers<PrincipalRadiiRSD>(py::class_<PointCloud<PrincipalRadiiRSD>, boost::shared_ptr<PointCloud<PrincipalRadiiRSD>>> &cls) {
    cls.def_property_readonly("r_min", &buffer<float, PrincipalRadiiRSD, 0, 1>);
    cls.def_property_readonly("r_max", &buffer<float, PrincipalRadiiRSD, 1, 1>);
}

template<>
void defineBuffers<Boundary>(py::class_<PointCloud<Boundary>, boost::shared_ptr<PointCloud<Boundary>>> &cls) {
    cls.def_property_readonly("boundary_point", &buffer<uint8_t, Boundary, 0, 1>);
}