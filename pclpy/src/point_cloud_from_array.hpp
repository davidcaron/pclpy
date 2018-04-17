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


