#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

template <typename BufferT, typename PointT, ssize_t OFFSET, ssize_t SIZE>
py::array buffer(boost::shared_ptr<PointCloud<PointT>> &cloud)
{
    std::vector<ssize_t> shape{static_cast<ssize_t>(cloud->size())};
    std::vector<ssize_t> strides{sizeof(PointT)};
    int ndim = 1;
    if (SIZE > 1)
    {
        shape.push_back(SIZE);
        strides.push_back(sizeof(BufferT));
        ndim = 2;
    }
    else if (SIZE < -1)
    {
        shape.push_back(-SIZE);
        strides.push_back(-sizeof(BufferT));
        ndim = 2;
    }
    else if (SIZE != 1)
    {
        throw std::runtime_error("Incompatible buffer size");
    }
    py::buffer_info buf = py::buffer_info(
        (BufferT *)(cloud->points.data()) + OFFSET, /* Pointer to buffer */
        sizeof(BufferT),                            /* Size of one scalar */
        py::format_descriptor<BufferT>::format(),   /* Python struct-style format descriptor */
        ndim,                                       /* Number of dimensions */
        shape,                                      /* Shape */
        strides                                     /* Strides (in bytes) for each index */
    );
    return py::array(buf);
}

template <typename T>
void defineBuffers(py::class_<PointCloud<T>, boost::shared_ptr<PointCloud<T>>> &cls){};

template <>
void defineBuffers<PointXYZ>(py::class_<PointCloud<PointXYZ>, boost::shared_ptr<PointCloud<PointXYZ>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointXYZ, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZ, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZ, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZ, 2, 1>);
}

template <>
void defineBuffers<PointXYZI>(py::class_<PointCloud<PointXYZI>, boost::shared_ptr<PointCloud<PointXYZI>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointXYZI, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZI, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZI, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZI, 2, 1>);
    cls.def_property_readonly("intensity", &buffer<float, PointXYZI, 4, 1>);
}

template <>
void defineBuffers<PointXYZL>(py::class_<PointCloud<PointXYZL>, boost::shared_ptr<PointCloud<PointXYZL>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointXYZL, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZL, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZL, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZL, 2, 1>);
    cls.def_property_readonly("label", &buffer<uint32_t, PointXYZL, 4, 1>);
}

template <>
void defineBuffers<Label>(py::class_<PointCloud<Label>, boost::shared_ptr<PointCloud<Label>>> &cls)
{
    cls.def_property_readonly("label", &buffer<uint32_t, Label, 0, 1>);
}

template <>
void defineBuffers<PointXYZRGBA>(py::class_<PointCloud<PointXYZRGBA>, boost::shared_ptr<PointCloud<PointXYZRGBA>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointXYZRGBA, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZRGBA, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZRGBA, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZRGBA, 2, 1>);
    cls.def_property_readonly("rgb", &buffer<uint8_t, PointXYZRGBA, 18, -3>);
    cls.def_property_readonly("argb", &buffer<uint8_t, PointXYZRGBA, 19, -4>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointXYZRGBA, 16, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointXYZRGBA, 17, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointXYZRGBA, 18, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointXYZRGBA, 19, 1>);
}

template <>
void defineBuffers<PointXYZRGB>(py::class_<PointCloud<PointXYZRGB>, boost::shared_ptr<PointCloud<PointXYZRGB>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointXYZRGB, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZRGB, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZRGB, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZRGB, 2, 1>);
    cls.def_property_readonly("rgb", &buffer<uint8_t, PointXYZRGB, 18, -3>);
    cls.def_property_readonly("argb", &buffer<uint8_t, PointXYZRGB, 19, -4>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointXYZRGB, 16, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointXYZRGB, 17, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointXYZRGB, 18, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointXYZRGB, 19, 1>);
}

template <>
void defineBuffers<PointXYZRGBL>(py::class_<PointCloud<PointXYZRGBL>, boost::shared_ptr<PointCloud<PointXYZRGBL>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointXYZRGBL, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZRGBL, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZRGBL, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZRGBL, 2, 1>);
    cls.def_property_readonly("rgb", &buffer<uint8_t, PointXYZRGBL, 18, -3>);
    cls.def_property_readonly("argb", &buffer<uint8_t, PointXYZRGBL, 19, -4>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointXYZRGBL, 16, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointXYZRGBL, 17, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointXYZRGBL, 18, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointXYZRGBL, 19, 1>);
    cls.def_property_readonly("label", &buffer<uint32_t, PointXYZRGBL, 5, 1>);
}

template <>
void defineBuffers<PointXYZHSV>(py::class_<PointCloud<PointXYZHSV>, boost::shared_ptr<PointCloud<PointXYZHSV>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointXYZHSV, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZHSV, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZHSV, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZHSV, 2, 1>);
    cls.def_property_readonly("hsv", &buffer<float, PointXYZHSV, 4, 3>);
    cls.def_property_readonly("h", &buffer<float, PointXYZHSV, 4, 1>);
    cls.def_property_readonly("s", &buffer<float, PointXYZHSV, 5, 1>);
    cls.def_property_readonly("v", &buffer<float, PointXYZHSV, 6, 1>);
}

template <>
void defineBuffers<PointXY>(py::class_<PointCloud<PointXY>, boost::shared_ptr<PointCloud<PointXY>>> &cls)
{
    cls.def_property_readonly("x", &buffer<float, PointXY, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXY, 1, 1>);
}

template <>
void defineBuffers<InterestPoint>(py::class_<PointCloud<InterestPoint>, boost::shared_ptr<PointCloud<InterestPoint>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, InterestPoint, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, InterestPoint, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, InterestPoint, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, InterestPoint, 2, 1>);
    cls.def_property_readonly("strength", &buffer<float, InterestPoint, 4, 1>);
}

template <>
void defineBuffers<Axis>(py::class_<PointCloud<Axis>, boost::shared_ptr<PointCloud<Axis>>> &cls)
{
    cls.def_property_readonly("normals", &buffer<float, Axis, 0, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, Axis, 0, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, Axis, 1, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, Axis, 2, 1>);
}

template <>
void defineBuffers<Normal>(py::class_<PointCloud<Normal>, boost::shared_ptr<PointCloud<Normal>>> &cls)
{
    cls.def_property_readonly("normals", &buffer<float, Normal, 0, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, Normal, 0, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, Normal, 1, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, Normal, 2, 1>);
    cls.def_property_readonly("curvature", &buffer<float, Normal, 4, 1>);
}

template <>
void defineBuffers<PointNormal>(py::class_<PointCloud<PointNormal>, boost::shared_ptr<PointCloud<PointNormal>>> &cls)
{
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

template <>
void defineBuffers<PointXYZRGBNormal>(py::class_<PointCloud<PointXYZRGBNormal>, boost::shared_ptr<PointCloud<PointXYZRGBNormal>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointXYZRGBNormal, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointXYZRGBNormal, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointXYZRGBNormal, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointXYZRGBNormal, 2, 1>);
    cls.def_property_readonly("normals", &buffer<float, PointXYZRGBNormal, 4, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, PointXYZRGBNormal, 4, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, PointXYZRGBNormal, 5, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, PointXYZRGBNormal, 6, 1>);
    cls.def_property_readonly("rgb", &buffer<uint8_t, PointXYZRGBNormal, 34, -3>);
    cls.def_property_readonly("argb", &buffer<uint8_t, PointXYZRGBNormal, 35, -4>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointXYZRGBNormal, 32, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointXYZRGBNormal, 33, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointXYZRGBNormal, 34, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointXYZRGBNormal, 35, 1>);
    cls.def_property_readonly("curvature", &buffer<float, PointXYZRGBNormal, 9, 1>);
}

template <>
void defineBuffers<PointXYZINormal>(py::class_<PointCloud<PointXYZINormal>, boost::shared_ptr<PointCloud<PointXYZINormal>>> &cls)
{
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

template <>
void defineBuffers<PointXYZLNormal>(py::class_<PointCloud<PointXYZLNormal>, boost::shared_ptr<PointCloud<PointXYZLNormal>>> &cls)
{
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

template <>
void defineBuffers<PointWithRange>(py::class_<PointCloud<PointWithRange>, boost::shared_ptr<PointCloud<PointWithRange>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointWithRange, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointWithRange, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointWithRange, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointWithRange, 2, 1>);
    cls.def_property_readonly("range", &buffer<float, PointWithRange, 4, 1>);
}

template <>
void defineBuffers<PointWithViewpoint>(py::class_<PointCloud<PointWithViewpoint>, boost::shared_ptr<PointCloud<PointWithViewpoint>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointWithViewpoint, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointWithViewpoint, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointWithViewpoint, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointWithViewpoint, 2, 1>);
    cls.def_property_readonly("vp", &buffer<float, PointWithViewpoint, 4, 3>);
    cls.def_property_readonly("vp_x", &buffer<float, PointWithViewpoint, 4, 1>);
    cls.def_property_readonly("vp_y", &buffer<float, PointWithViewpoint, 5, 1>);
    cls.def_property_readonly("vp_z", &buffer<float, PointWithViewpoint, 6, 1>);
}

template <>
void defineBuffers<MomentInvariants>(py::class_<PointCloud<MomentInvariants>, boost::shared_ptr<PointCloud<MomentInvariants>>> &cls)
{
    cls.def_property_readonly("j1", &buffer<float, MomentInvariants, 0, 1>);
    cls.def_property_readonly("j2", &buffer<float, MomentInvariants, 1, 1>);
    cls.def_property_readonly("j3", &buffer<float, MomentInvariants, 2, 1>);
}

template <>
void defineBuffers<PrincipalRadiiRSD>(py::class_<PointCloud<PrincipalRadiiRSD>, boost::shared_ptr<PointCloud<PrincipalRadiiRSD>>> &cls)
{
    cls.def_property_readonly("r_min", &buffer<float, PrincipalRadiiRSD, 0, 1>);
    cls.def_property_readonly("r_max", &buffer<float, PrincipalRadiiRSD, 1, 1>);
}

template <>
void defineBuffers<Boundary>(py::class_<PointCloud<Boundary>, boost::shared_ptr<PointCloud<Boundary>>> &cls)
{
    cls.def_property_readonly("boundary_point", &buffer<uint8_t, Boundary, 0, 1>);
}

template <>
void defineBuffers<PrincipalCurvatures>(py::class_<PointCloud<PrincipalCurvatures>, boost::shared_ptr<PointCloud<PrincipalCurvatures>>> &cls)
{
    cls.def_property_readonly("principal_curvature", &buffer<float, PrincipalCurvatures, 0, 3>);
    cls.def_property_readonly("principal_curvature_x", &buffer<float, PrincipalCurvatures, 0, 1>);
    cls.def_property_readonly("principal_curvature_y", &buffer<float, PrincipalCurvatures, 1, 1>);
    cls.def_property_readonly("principal_curvature_z", &buffer<float, PrincipalCurvatures, 2, 1>);
    cls.def_property_readonly("pc1", &buffer<float, PrincipalCurvatures, 3, 1>);
    cls.def_property_readonly("pc2", &buffer<float, PrincipalCurvatures, 4, 1>);
}

template <>
void defineBuffers<PFHSignature125>(py::class_<PointCloud<PFHSignature125>, boost::shared_ptr<PointCloud<PFHSignature125>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, PFHSignature125, 0, 125>);
}

template <>
void defineBuffers<PFHRGBSignature250>(py::class_<PointCloud<PFHRGBSignature250>, boost::shared_ptr<PointCloud<PFHRGBSignature250>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, PFHRGBSignature250, 0, 250>);
}

template <>
void defineBuffers<PPFSignature>(py::class_<PointCloud<PPFSignature>, boost::shared_ptr<PointCloud<PPFSignature>>> &cls)
{
    cls.def_property_readonly("f1", &buffer<float, PPFSignature, 0, 1>);
    cls.def_property_readonly("f2", &buffer<float, PPFSignature, 1, 1>);
    cls.def_property_readonly("f3", &buffer<float, PPFSignature, 2, 1>);
    cls.def_property_readonly("f4", &buffer<float, PPFSignature, 3, 1>);
    cls.def_property_readonly("alpha_m", &buffer<float, PPFSignature, 4, 1>);
}

template <>
void defineBuffers<CPPFSignature>(py::class_<PointCloud<CPPFSignature>, boost::shared_ptr<PointCloud<CPPFSignature>>> &cls)
{
    cls.def_property_readonly("f1", &buffer<float, CPPFSignature, 0, 1>);
    cls.def_property_readonly("f2", &buffer<float, CPPFSignature, 1, 1>);
    cls.def_property_readonly("f3", &buffer<float, CPPFSignature, 2, 1>);
    cls.def_property_readonly("f4", &buffer<float, CPPFSignature, 3, 1>);
    cls.def_property_readonly("f5", &buffer<float, CPPFSignature, 4, 1>);
    cls.def_property_readonly("f6", &buffer<float, CPPFSignature, 5, 1>);
    cls.def_property_readonly("f7", &buffer<float, CPPFSignature, 6, 1>);
    cls.def_property_readonly("f8", &buffer<float, CPPFSignature, 7, 1>);
    cls.def_property_readonly("f9", &buffer<float, CPPFSignature, 8, 1>);
    cls.def_property_readonly("f10", &buffer<float, CPPFSignature, 9, 1>);
    cls.def_property_readonly("alpha_m", &buffer<float, CPPFSignature, 10, 1>);
}

template <>
void defineBuffers<PPFRGBSignature>(py::class_<PointCloud<PPFRGBSignature>, boost::shared_ptr<PointCloud<PPFRGBSignature>>> &cls)
{
    cls.def_property_readonly("f1", &buffer<float, PPFRGBSignature, 0, 1>);
    cls.def_property_readonly("f2", &buffer<float, PPFRGBSignature, 1, 1>);
    cls.def_property_readonly("f3", &buffer<float, PPFRGBSignature, 2, 1>);
    cls.def_property_readonly("f4", &buffer<float, PPFRGBSignature, 3, 1>);
    cls.def_property_readonly("r_ratio", &buffer<float, PPFRGBSignature, 4, 1>);
    cls.def_property_readonly("g_ratio", &buffer<float, PPFRGBSignature, 5, 1>);
    cls.def_property_readonly("b_ratio", &buffer<float, PPFRGBSignature, 6, 1>);
    cls.def_property_readonly("alpha_m", &buffer<float, PPFRGBSignature, 7, 1>);
}

template <>
void defineBuffers<NormalBasedSignature12>(py::class_<PointCloud<NormalBasedSignature12>, boost::shared_ptr<PointCloud<NormalBasedSignature12>>> &cls)
{
    cls.def_property_readonly("values", &buffer<float, NormalBasedSignature12, 0, 12>);
}

template <>
void defineBuffers<FPFHSignature33>(py::class_<PointCloud<FPFHSignature33>, boost::shared_ptr<PointCloud<FPFHSignature33>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, FPFHSignature33, 0, 33>);
}

template <>
void defineBuffers<VFHSignature308>(py::class_<PointCloud<VFHSignature308>, boost::shared_ptr<PointCloud<VFHSignature308>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, VFHSignature308, 0, 308>);
}

template <>
void defineBuffers<GASDSignature512>(py::class_<PointCloud<GASDSignature512>, boost::shared_ptr<PointCloud<GASDSignature512>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, GASDSignature512, 0, 512>);
}

template <>
void defineBuffers<GASDSignature984>(py::class_<PointCloud<GASDSignature984>, boost::shared_ptr<PointCloud<GASDSignature984>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, GASDSignature984, 0, 984>);
}

template <>
void defineBuffers<GASDSignature7992>(py::class_<PointCloud<GASDSignature7992>, boost::shared_ptr<PointCloud<GASDSignature7992>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, GASDSignature7992, 0, 7992>);
}

template <>
void defineBuffers<GRSDSignature21>(py::class_<PointCloud<GRSDSignature21>, boost::shared_ptr<PointCloud<GRSDSignature21>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, GRSDSignature21, 0, 21>);
}

template <>
void defineBuffers<ESFSignature640>(py::class_<PointCloud<ESFSignature640>, boost::shared_ptr<PointCloud<ESFSignature640>>> &cls)
{
    cls.def_property_readonly("histogram", &buffer<float, ESFSignature640, 0, 640>);
}

template <>
void defineBuffers<BRISKSignature512>(py::class_<PointCloud<BRISKSignature512>, boost::shared_ptr<PointCloud<BRISKSignature512>>> &cls)
{
    cls.def_property_readonly("scale", &buffer<float, BRISKSignature512, 0, 1>);
    cls.def_property_readonly("orientation", &buffer<float, BRISKSignature512, 1, 1>);
    cls.def_property_readonly("descriptor", &buffer<unsigned char, BRISKSignature512, 8, 64>);
}

template <>
void defineBuffers<Narf36>(py::class_<PointCloud<Narf36>, boost::shared_ptr<PointCloud<Narf36>>> &cls)
{
    cls.def_property_readonly("x", &buffer<float, Narf36, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, Narf36, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, Narf36, 2, 1>);
    cls.def_property_readonly("roll", &buffer<float, Narf36, 3, 1>);
    cls.def_property_readonly("pitch", &buffer<float, Narf36, 4, 1>);
    cls.def_property_readonly("yaw", &buffer<float, Narf36, 5, 1>);
    cls.def_property_readonly("descriptor", &buffer<float, Narf36, 6, 36>);
}

template <>
void defineBuffers<IntensityGradient>(py::class_<PointCloud<IntensityGradient>, boost::shared_ptr<PointCloud<IntensityGradient>>> &cls)
{
    cls.def_property_readonly("gradient", &buffer<float, IntensityGradient, 0, 3>);
    cls.def_property_readonly("gradient_x", &buffer<float, IntensityGradient, 0, 1>);
    cls.def_property_readonly("gradient_y", &buffer<float, IntensityGradient, 1, 1>);
    cls.def_property_readonly("gradient_z", &buffer<float, IntensityGradient, 2, 1>);
}

template <>
void defineBuffers<PointWithScale>(py::class_<PointCloud<PointWithScale>, boost::shared_ptr<PointCloud<PointWithScale>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointWithScale, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointWithScale, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointWithScale, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointWithScale, 2, 1>);
    cls.def_property_readonly("scale", &buffer<float, PointWithScale, 4, 1>);
    cls.def_property_readonly("size", &buffer<float, PointWithScale, 4, 1>);
    cls.def_property_readonly("angle", &buffer<float, PointWithScale, 5, 1>);
    cls.def_property_readonly("response", &buffer<float, PointWithScale, 6, 1>);
    cls.def_property_readonly("octave", &buffer<int, PointWithScale, 7, 1>);
}

template <>
void defineBuffers<PointSurfel>(py::class_<PointCloud<PointSurfel>, boost::shared_ptr<PointCloud<PointSurfel>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointSurfel, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointSurfel, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointSurfel, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointSurfel, 2, 1>);
    cls.def_property_readonly("normals", &buffer<float, PointSurfel, 4, 3>);
    cls.def_property_readonly("normal_x", &buffer<float, PointSurfel, 4, 1>);
    cls.def_property_readonly("normal_y", &buffer<float, PointSurfel, 5, 1>);
    cls.def_property_readonly("normal_z", &buffer<float, PointSurfel, 6, 1>);
    cls.def_property_readonly("rgb", &buffer<uint8_t, PointSurfel, 34, -3>);
    cls.def_property_readonly("argb", &buffer<uint8_t, PointSurfel, 35, -4>);
    cls.def_property_readonly("b", &buffer<uint8_t, PointSurfel, 32, 1>);
    cls.def_property_readonly("g", &buffer<uint8_t, PointSurfel, 33, 1>);
    cls.def_property_readonly("r", &buffer<uint8_t, PointSurfel, 34, 1>);
    cls.def_property_readonly("a", &buffer<uint8_t, PointSurfel, 35, 1>);
    cls.def_property_readonly("radius", &buffer<float, PointSurfel, 9, 1>);
    cls.def_property_readonly("confidence", &buffer<float, PointSurfel, 10, 1>);
    cls.def_property_readonly("curvature", &buffer<float, PointSurfel, 11, 1>);
}

template <>
void defineBuffers<ShapeContext1980>(py::class_<PointCloud<ShapeContext1980>, boost::shared_ptr<PointCloud<ShapeContext1980>>> &cls)
{
    cls.def_property_readonly("descriptor", &buffer<float, ShapeContext1980, 0, 1980>);
    cls.def_property_readonly("rf", &buffer<float, ShapeContext1980, 1980, 9>);
}

template <>
void defineBuffers<UniqueShapeContext1960>(py::class_<PointCloud<UniqueShapeContext1960>, boost::shared_ptr<PointCloud<UniqueShapeContext1960>>> &cls)
{
    cls.def_property_readonly("descriptor", &buffer<float, UniqueShapeContext1960, 0, 1960>);
    cls.def_property_readonly("rf", &buffer<float, UniqueShapeContext1960, 1960, 9>);
}

template <>
void defineBuffers<SHOT352>(py::class_<PointCloud<SHOT352>, boost::shared_ptr<PointCloud<SHOT352>>> &cls)
{
    cls.def_property_readonly("descriptor", &buffer<float, SHOT352, 0, 352>);
    cls.def_property_readonly("rf", &buffer<float, SHOT352, 352, 9>);
}

template <>
void defineBuffers<SHOT1344>(py::class_<PointCloud<SHOT1344>, boost::shared_ptr<PointCloud<SHOT1344>>> &cls)
{
    cls.def_property_readonly("descriptor", &buffer<float, SHOT1344, 0, 1344>);
    cls.def_property_readonly("rf", &buffer<float, SHOT1344, 1344, 9>);
}

template <>
void defineBuffers<PointUV>(py::class_<PointCloud<PointUV>, boost::shared_ptr<PointCloud<PointUV>>> &cls)
{
    cls.def_property_readonly("u", &buffer<float, PointUV, 0, 1>);
    cls.def_property_readonly("v", &buffer<float, PointUV, 1, 1>);
}

template <>
void defineBuffers<ReferenceFrame>(py::class_<PointCloud<ReferenceFrame>, boost::shared_ptr<PointCloud<ReferenceFrame>>> &cls)
{
    cls.def_property_readonly("rf", &buffer<float, ReferenceFrame, 0, 9>);
    cls.def_property_readonly("x_axis", &buffer<float, ReferenceFrame, 0, 3>);
    cls.def_property_readonly("y_axis", &buffer<float, ReferenceFrame, 3, 3>);
    cls.def_property_readonly("z_axis", &buffer<float, ReferenceFrame, 6, 3>);
}

template <>
void defineBuffers<PointDEM>(py::class_<PointCloud<PointDEM>, boost::shared_ptr<PointCloud<PointDEM>>> &cls)
{
    cls.def_property_readonly("xyz", &buffer<float, PointDEM, 0, 3>);
    cls.def_property_readonly("x", &buffer<float, PointDEM, 0, 1>);
    cls.def_property_readonly("y", &buffer<float, PointDEM, 1, 1>);
    cls.def_property_readonly("z", &buffer<float, PointDEM, 2, 1>);
    cls.def_property_readonly("intensity", &buffer<float, PointDEM, 4, 1>);
    cls.def_property_readonly("intensity_variance", &buffer<float, PointDEM, 5, 1>);
    cls.def_property_readonly("height_variance", &buffer<float, PointDEM, 6, 1>);
}