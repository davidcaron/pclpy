#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>

#include <pcl/point_representation.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


template<typename T>
boost::shared_ptr<PointCloud<T>> fromArray(py::array_t<float> &points)
{
};

template<typename T>
boost::shared_ptr<PointCloud<T>> fromArrayRGB(py::array_t<float> &points,
                                                   py::array_t<uint8_t> &rgb)
{
};

template<>
PointCloud<PointXYZ>::Ptr fromArray<PointXYZ>(py::array_t<float> &points) {
    PointCloud<PointXYZ>::Ptr c (new PointCloud<PointXYZ>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointXYZ *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
    }
    return c;
}


template<>
PointCloud<PointXYZI>::Ptr fromArray<PointXYZI>(py::array_t<float> &points) {
    PointCloud<PointXYZI>::Ptr c (new PointCloud<PointXYZI>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointXYZI *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->intensity = r(i, 3);
    }
    return c;
}

template<>
PointCloud<PointXYZL>::Ptr fromArray<PointXYZL>(py::array_t<float> &points) {
    PointCloud<PointXYZL>::Ptr c (new PointCloud<PointXYZL>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointXYZL *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->label = static_cast<uint32_t>(r(i, 3));
    }
    return c;
}

template<>
PointCloud<Label>::Ptr fromArray<Label>(py::array_t<float> &points) {
    PointCloud<Label>::Ptr c (new PointCloud<Label>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    Label *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->label = static_cast<uint32_t>(r(i, 0));
    }
    return c;
}


template<>
PointCloud<PointXYZRGBA>::Ptr fromArrayRGB<PointXYZRGBA>(py::array_t<float> &points,
                                                                        py::array_t<uint8_t> &rgb) {
    PointCloud<PointXYZRGBA>::Ptr c (new PointCloud<PointXYZRGBA>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    auto r2 = rgb.unchecked<2>();
    PointXYZRGBA *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->r = r2(i, 0);
        pt->g = r2(i, 1);
        pt->b = r2(i, 2);
        pt->a = 255;
    }
    return c;
}

template<>
PointCloud<PointXYZRGB>::Ptr fromArrayRGB<PointXYZRGB>(py::array_t<float> &points,
                                                                        py::array_t<uint8_t> &rgb) {
    PointCloud<PointXYZRGB>::Ptr c (new PointCloud<PointXYZRGB>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    auto r2 = rgb.unchecked<2>();
    PointXYZRGB *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->r = r2(i, 0);
        pt->g = r2(i, 1);
        pt->b = r2(i, 2);
        pt->a = 255;
    }
    return c;
}

template<>
PointCloud<PointXYZRGBL>::Ptr fromArrayRGB<PointXYZRGBL>(py::array_t<float> &points,
                                                                        py::array_t<uint8_t> &rgb) {
    PointCloud<PointXYZRGBL>::Ptr c (new PointCloud<PointXYZRGBL>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    auto r2 = rgb.unchecked<2>();
    PointXYZRGBL *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->label = static_cast<uint32_t>(r(i, 3));
        pt->r = r2(i, 0);
        pt->g = r2(i, 1);
        pt->b = r2(i, 2);
        pt->a = 255;
    }
    return c;
}

template<>
PointCloud<PointXYZHSV>::Ptr fromArray<PointXYZHSV>(py::array_t<float> &points) {
    PointCloud<PointXYZHSV>::Ptr c (new PointCloud<PointXYZHSV>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointXYZHSV *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->h = r(i, 3);
        pt->s = r(i, 4);
        pt->v = r(i, 5);
    }
    return c;
}

template<>
PointCloud<PointXY>::Ptr fromArray<PointXY>(py::array_t<float> &points) {
    PointCloud<PointXY>::Ptr c (new PointCloud<PointXY>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointXY *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
    }
    return c;
}


template<>
PointCloud<InterestPoint>::Ptr fromArray<InterestPoint>(py::array_t<float> &points) {
    PointCloud<InterestPoint>::Ptr c (new PointCloud<InterestPoint>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    InterestPoint *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->strength = r(i, 3);
    }
    return c;
}

template<>
PointCloud<Axis>::Ptr fromArray<Axis>(py::array_t<float> &points) {
    PointCloud<Axis>::Ptr c (new PointCloud<Axis>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    Axis *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->normal_x = r(i, 0);
        pt->normal_y = r(i, 1);
        pt->normal_z = r(i, 2);
    }
    return c;
}

template<>
PointCloud<Normal>::Ptr fromArray<Normal>(py::array_t<float> &points) {
    PointCloud<Normal>::Ptr c (new PointCloud<Normal>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    Normal *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->normal_x = r(i, 0);
        pt->normal_y = r(i, 1);
        pt->normal_z = r(i, 2);
        pt->curvature = r(i, 3);
    }
    return c;
}

template<>
PointCloud<PointNormal>::Ptr fromArray<PointNormal>(py::array_t<float> &points) {
    PointCloud<PointNormal>::Ptr c (new PointCloud<PointNormal>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointNormal *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->normal_x = r(i, 3);
        pt->normal_y = r(i, 4);
        pt->normal_z = r(i, 5);
        pt->curvature = r(i, 6);
    }
    return c;
}

template<>
PointCloud<PointXYZRGBNormal>::Ptr fromArrayRGB<PointXYZRGBNormal>(py::array_t<float> &points,
                                                                        py::array_t<uint8_t> &rgb) {
    PointCloud<PointXYZRGBNormal>::Ptr c (new PointCloud<PointXYZRGBNormal>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    auto r2 = rgb.unchecked<2>();
    PointXYZRGBNormal *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->normal_x = r(i, 3);
        pt->normal_y = r(i, 4);
        pt->normal_z = r(i, 5);
        pt->curvature = r(i, 6);
        pt->r = r2(i, 0);
        pt->g = r2(i, 1);
        pt->b = r2(i, 2);
        pt->a = 255;
    }
    return c;
}

template<>
PointCloud<PointXYZINormal>::Ptr fromArray<PointXYZINormal>(py::array_t<float> &points) {
    PointCloud<PointXYZINormal>::Ptr c (new PointCloud<PointXYZINormal>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointXYZINormal *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->intensity = r(i, 3);
        pt->normal_x = r(i, 4);
        pt->normal_y = r(i, 5);
        pt->normal_z = r(i, 6);
        pt->curvature = r(i, 7);
    }
    return c;
}

template<>
PointCloud<PointXYZLNormal>::Ptr fromArray<PointXYZLNormal>(py::array_t<float> &points) {
    PointCloud<PointXYZLNormal>::Ptr c (new PointCloud<PointXYZLNormal>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointXYZLNormal *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->label = static_cast<uint32_t>(r(i, 3));
        pt->normal_x = r(i, 4);
        pt->normal_y = r(i, 5);
        pt->normal_z = r(i, 6);
        pt->curvature = r(i, 7);
    }
    return c;
}


template<>
PointCloud<PointWithRange>::Ptr fromArray<PointWithRange>(py::array_t<float> &points) {
    PointCloud<PointWithRange>::Ptr c (new PointCloud<PointWithRange>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointWithRange *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->range = r(i, 3);
    }
    return c;
}

template<>
PointCloud<PointWithViewpoint>::Ptr fromArray<PointWithViewpoint>(py::array_t<float> &points) {
    PointCloud<PointWithViewpoint>::Ptr c (new PointCloud<PointWithViewpoint>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointWithViewpoint *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->vp_x = r(i, 3);
        pt->vp_y = r(i, 4);
        pt->vp_z = r(i, 5);
    }
    return c;
}

template<>
PointCloud<MomentInvariants>::Ptr fromArray<MomentInvariants>(py::array_t<float> &points) {
    PointCloud<MomentInvariants>::Ptr c (new PointCloud<MomentInvariants>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    MomentInvariants *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->j1 = r(i, 0);
        pt->j2 = r(i, 1);
        pt->j3 = r(i, 2);
    }
    return c;
}

template<>
PointCloud<PrincipalRadiiRSD>::Ptr fromArray<PrincipalRadiiRSD>(py::array_t<float> &points) {
    PointCloud<PrincipalRadiiRSD>::Ptr c (new PointCloud<PrincipalRadiiRSD>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PrincipalRadiiRSD *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->r_min = r(i, 0);
        pt->r_max = r(i, 1);
    }
    return c;
}

template<>
PointCloud<Boundary>::Ptr fromArray<Boundary>(py::array_t<float> &points) {
    PointCloud<Boundary>::Ptr c (new PointCloud<Boundary>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    Boundary *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->boundary_point = static_cast<uint8_t>(r(i, 0));
    }
    return c;
}
