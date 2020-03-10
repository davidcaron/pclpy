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

template<>
PointCloud<PrincipalCurvatures>::Ptr fromArray<PrincipalCurvatures>(py::array_t<float> &points) {
    PointCloud<PrincipalCurvatures>::Ptr c (new PointCloud<PrincipalCurvatures>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PrincipalCurvatures *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->principal_curvature_x = r(i, 0);
        pt->principal_curvature_y = r(i, 1);
        pt->principal_curvature_z = r(i, 2);
        pt->pc1 = r(i, 3);
        pt->pc2 = r(i, 4);
    }
    return c;
}

template<>
PointCloud<PFHSignature125>::Ptr fromArray<PFHSignature125>(py::array_t<float> &points) {
    PointCloud<PFHSignature125>::Ptr c (new PointCloud<PFHSignature125>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PFHSignature125 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 125; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}

template<>
PointCloud<PFHRGBSignature250>::Ptr fromArray<PFHRGBSignature250>(py::array_t<float> &points) {
    PointCloud<PFHRGBSignature250>::Ptr c (new PointCloud<PFHRGBSignature250>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PFHRGBSignature250 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 250; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}

template<>
PointCloud<PPFSignature>::Ptr fromArray<PPFSignature>(py::array_t<float> &points) {
    PointCloud<PPFSignature>::Ptr c (new PointCloud<PPFSignature>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PPFSignature *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->f1 = r(i, 0);
        pt->f2 = r(i, 1);
        pt->f3 = r(i, 2);
        pt->f4 = r(i, 3);
        pt->alpha_m = r(i, 4);
    }
    return c;
}

template<>
PointCloud<CPPFSignature>::Ptr fromArray<CPPFSignature>(py::array_t<float> &points) {
    PointCloud<CPPFSignature>::Ptr c (new PointCloud<CPPFSignature>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    CPPFSignature *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->f1 = r(i, 0);
        pt->f2 = r(i, 1);
        pt->f3 = r(i, 2);
        pt->f4 = r(i, 3);
        pt->f5 = r(i, 4);
        pt->f6 = r(i, 5);
        pt->f7 = r(i, 6);
        pt->f8 = r(i, 7);
        pt->f9 = r(i, 8);
        pt->f10 = r(i, 9);
        pt->alpha_m = r(i, 10);
    }
    return c;
}

template<>
PointCloud<PPFRGBSignature>::Ptr fromArray<PPFRGBSignature>(py::array_t<float> &points) {
    PointCloud<PPFRGBSignature>::Ptr c (new PointCloud<PPFRGBSignature>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PPFRGBSignature *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->f1 = r(i, 0);
        pt->f2 = r(i, 1);
        pt->f3 = r(i, 2);
        pt->f4 = r(i, 3);
        pt->r_ratio = r(i, 4);
        pt->g_ratio = r(i, 5);
        pt->b_ratio = r(i, 6);
        pt->alpha_m = r(i, 7);
    }
    return c;
}

template<>
PointCloud<NormalBasedSignature12>::Ptr fromArray<NormalBasedSignature12>(py::array_t<float> &points) {
    PointCloud<NormalBasedSignature12>::Ptr c (new PointCloud<NormalBasedSignature12>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    NormalBasedSignature12 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 12; d++)
        {
            pt->values[d] = r(i, d);
        }
    }
    return c;
}

template<>
PointCloud<FPFHSignature33>::Ptr fromArray<FPFHSignature33>(py::array_t<float> &points) {
    PointCloud<FPFHSignature33>::Ptr c (new PointCloud<FPFHSignature33>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    FPFHSignature33 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 33; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}

template<>
PointCloud<VFHSignature308>::Ptr fromArray<VFHSignature308>(py::array_t<float> &points) {
    PointCloud<VFHSignature308>::Ptr c (new PointCloud<VFHSignature308>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    VFHSignature308 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 308; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}


template<>
PointCloud<GASDSignature512>::Ptr fromArray<GASDSignature512>(py::array_t<float> &points) {
    PointCloud<GASDSignature512>::Ptr c (new PointCloud<GASDSignature512>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    GASDSignature512 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 512; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}


template<>
PointCloud<GASDSignature984>::Ptr fromArray<GASDSignature984>(py::array_t<float> &points) {
    PointCloud<GASDSignature984>::Ptr c (new PointCloud<GASDSignature984>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    GASDSignature984 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 984; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}


template<>
PointCloud<GASDSignature7992>::Ptr fromArray<GASDSignature7992>(py::array_t<float> &points) {
    PointCloud<GASDSignature7992>::Ptr c (new PointCloud<GASDSignature7992>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    GASDSignature7992 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 7992; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}


template<>
PointCloud<GRSDSignature21>::Ptr fromArray<GRSDSignature21>(py::array_t<float> &points) {
    PointCloud<GRSDSignature21>::Ptr c (new PointCloud<GRSDSignature21>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    GRSDSignature21 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 21; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}

template<>
PointCloud<ESFSignature640>::Ptr fromArray<ESFSignature640>(py::array_t<float> &points) {
    PointCloud<ESFSignature640>::Ptr c (new PointCloud<ESFSignature640>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    ESFSignature640 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 640; d++)
        {
            pt->histogram[d] = r(i, d);
        }
    }
    return c;
}

template<>
PointCloud<BRISKSignature512>::Ptr fromArray<BRISKSignature512>(py::array_t<float> &points) {
    PointCloud<BRISKSignature512>::Ptr c (new PointCloud<BRISKSignature512>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    BRISKSignature512 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->scale = r(i, 0);
        pt->orientation = r(i, 1);
        for (ssize_t d = 0; d < 64; d++)
        {
            pt->descriptor[d] = static_cast<unsigned char>( r(i, 2 + d) );
        }
    }
    return c;
}

template<>
PointCloud<Narf36>::Ptr fromArray<Narf36>(py::array_t<float> &points) {
    PointCloud<Narf36>::Ptr c (new PointCloud<Narf36>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    Narf36 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->roll = r(i, 3);
        pt->pitch = r(i, 4);
        pt->yaw = r(i, 5);
        for (ssize_t d = 0; d < 36; d++)
        {
            pt->descriptor[d] = r(i, 6 + d);
        }
    }
    return c;
}

template<>
PointCloud<IntensityGradient>::Ptr fromArray<IntensityGradient>(py::array_t<float> &points) {
    PointCloud<IntensityGradient>::Ptr c (new PointCloud<IntensityGradient>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    IntensityGradient *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->gradient_x = r(i, 0);
        pt->gradient_y = r(i, 1);
        pt->gradient_z = r(i, 2);
    }
    return c;
}

template<>
PointCloud<PointWithScale>::Ptr fromArray<PointWithScale>(py::array_t<float> &points) {
    PointCloud<PointWithScale>::Ptr c (new PointCloud<PointWithScale>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointWithScale *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->scale = r(i, 3);
        pt->angle = r(i, 4);
        pt->response = r(i, 5);
        pt->octave = static_cast<int>(r(i, 6));
    }
    return c;
}

template<>
PointCloud<PointSurfel>::Ptr fromArrayRGB<PointSurfel>(py::array_t<float> &points,
                                                       py::array_t<uint8_t> &rgb) {
    PointCloud<PointSurfel>::Ptr c (new PointCloud<PointSurfel>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    auto r2 = rgb.unchecked<2>();
    PointSurfel *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->normal_x = r(i, 3);
        pt->normal_y = r(i, 4);
        pt->normal_z = r(i, 5);
        pt->radius = r(i, 6);
        pt->confidence = r(i, 7);
        pt->curvature = r(i, 8);
        pt->r = r2(i, 0);
        pt->g = r2(i, 1);
        pt->b = r2(i, 2);
        pt->a = 255;
    }
    return c;
}

template<>
PointCloud<ShapeContext1980>::Ptr fromArray<ShapeContext1980>(py::array_t<float> &points) {
    PointCloud<ShapeContext1980>::Ptr c (new PointCloud<ShapeContext1980>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    ShapeContext1980 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 1980; d++)
        {
            pt->descriptor[d] = r(i, d);
        }
        for (ssize_t d = 0; d < 9; d++)
        {
            pt->rf[d] = r(i, 1980 + d);
        }
    }
    return c;
}

template<>
PointCloud<UniqueShapeContext1960>::Ptr fromArray<UniqueShapeContext1960>(py::array_t<float> &points) {
    PointCloud<UniqueShapeContext1960>::Ptr c (new PointCloud<UniqueShapeContext1960>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    UniqueShapeContext1960 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 1960; d++)
        {
            pt->descriptor[d] = r(i, d);
        }
        for (ssize_t d = 0; d < 9; d++)
        {
            pt->rf[d] = r(i, 1960 + d);
        }
    }
    return c;
}

template<>
PointCloud<SHOT352>::Ptr fromArray<SHOT352>(py::array_t<float> &points) {
    PointCloud<SHOT352>::Ptr c (new PointCloud<SHOT352>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    SHOT352 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 352; d++)
        {
            pt->descriptor[d] = r(i, d);
        }
        for (ssize_t d = 0; d < 9; d++)
        {
            pt->rf[d] = r(i, 352 + d);
        }
    }
    return c;
}

template<>
PointCloud<SHOT1344>::Ptr fromArray<SHOT1344>(py::array_t<float> &points) {
    PointCloud<SHOT1344>::Ptr c (new PointCloud<SHOT1344>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    SHOT1344 *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 1344; d++)
        {
            pt->descriptor[d] = r(i, d);
        }
        for (ssize_t d = 0; d < 9; d++)
        {
            pt->rf[d] = r(i, 1344 + d);
        }
    }
    return c;
}

template<>
PointCloud<PointUV>::Ptr fromArray<PointUV>(py::array_t<float> &points) {
    PointCloud<PointUV>::Ptr c (new PointCloud<PointUV>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointUV *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->u = r(i, 0);
        pt->v = r(i, 1);
    }
    return c;
}

template<>
PointCloud<ReferenceFrame>::Ptr fromArray<ReferenceFrame>(py::array_t<float> &points) {
    PointCloud<ReferenceFrame>::Ptr c (new PointCloud<ReferenceFrame>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    ReferenceFrame *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        for (ssize_t d = 0; d < 9; d++)
        {
            pt->rf[d] = r(i, d);
        }
    }
    return c;
}

template<>
PointCloud<PointDEM>::Ptr fromArray<PointDEM>(py::array_t<float> &points) {
    PointCloud<PointDEM>::Ptr c (new PointCloud<PointDEM>);
    c->resize(points.shape(0));
    auto r = points.unchecked<2>();
    PointDEM *pt;
    for (ssize_t i = 0; i < r.shape(0); i++)
    {
        pt = &c->at(i);
        pt->x = r(i, 0);
        pt->y = r(i, 1);
        pt->z = r(i, 2);
        pt->intensity = r(i, 3);
        pt->intensity_variance = r(i, 4);
        pt->height_variance = r(i, 5);
    }
    return c;
}