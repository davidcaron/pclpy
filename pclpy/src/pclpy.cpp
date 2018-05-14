
#pragma warning (disable : 4367)
#pragma warning (disable : 4267)

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <pcl/point_representation.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);

#include "make_opaque_vectors.hpp"  // must be first for PYBIND11_MAKE_OPAQUE to work
#include "eigen_bind.hpp"
#include "generated_modules/__main_loader.hpp"
#include "point_cloud_buffers.hpp"
#include "point_cloud_from_array.hpp"
#include "vector_classes.hpp"
#include "point_types.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

template <typename T>
void definePointCloudBuffers(py::module &m, const char* suffix) {
    using PointCloud = pcl::PointCloud<T>;
    using Type = py::class_<PointCloud, boost::shared_ptr<PointCloud>>;
    auto pc = static_cast<Type>(m.attr(suffix));
    pc.def_static("from_array", &fromArray<T>);
    defineBuffers<T>(pc);
}

template <typename T>
void definePointCloudBuffersRGB(py::module &m, const char* suffix) {
    using PointCloud = pcl::PointCloud<T>;
    using Type = py::class_<PointCloud, boost::shared_ptr<PointCloud>>;
    auto pc = static_cast<Type>(m.attr(suffix));
    pc.def_static("from_array", &fromArrayRGB<T>);
    defineBuffers<T>(pc);
}

PYBIND11_MODULE(pcl, m) {
    m.doc() = "PCL python bindings";

    definePointTypes(m);

    py::module m_vector = m.def_submodule("vectors", "Submodule for vectors");
    defineEigenClasses(m_vector);
    defineVectorClasses(m_vector);

    defineClasses(m);

    py::module pc = m.attr("PointCloud");
    definePointCloudBuffers<PointXYZ>(pc, "PointXYZ");
    definePointCloudBuffers<PointXYZI>(pc, "PointXYZI");
    definePointCloudBuffers<PointXYZL>(pc, "PointXYZL");
    definePointCloudBuffers<Label>(pc, "Label");
    definePointCloudBuffersRGB<PointXYZRGBA>(pc, "PointXYZRGBA");
    definePointCloudBuffersRGB<PointXYZRGB>(pc, "PointXYZRGB");
    definePointCloudBuffersRGB<PointXYZRGBL>(pc, "PointXYZRGBL");
    definePointCloudBuffers<PointXYZHSV>(pc, "PointXYZHSV");
    definePointCloudBuffers<PointXY>(pc, "PointXY");
    definePointCloudBuffers<InterestPoint>(pc, "InterestPoint");
    definePointCloudBuffers<Axis>(pc, "Axis");
    definePointCloudBuffers<Normal>(pc, "Normal");
    definePointCloudBuffers<PointNormal>(pc, "PointNormal");
    definePointCloudBuffersRGB<PointXYZRGBNormal>(pc, "PointXYZRGBNormal");
    definePointCloudBuffers<PointXYZINormal>(pc, "PointXYZINormal");
    definePointCloudBuffers<PointXYZLNormal>(pc, "PointXYZLNormal");
    definePointCloudBuffers<PointWithRange>(pc, "PointWithRange");
    definePointCloudBuffers<PointWithViewpoint>(pc, "PointWithViewpoint");
    definePointCloudBuffers<MomentInvariants>(pc, "MomentInvariants");
    definePointCloudBuffers<PrincipalRadiiRSD>(pc, "PrincipalRadiiRSD");
    definePointCloudBuffers<PrincipalRadiiRSD>(pc, "PrincipalRadiiRSD");
    definePointCloudBuffers<Boundary>(pc, "Boundary");
    definePointCloudBuffers<PrincipalCurvatures>(pc, "PrincipalCurvatures");
    definePointCloudBuffers<PFHSignature125>(pc, "PFHSignature125");
    definePointCloudBuffers<PFHRGBSignature250>(pc, "PFHRGBSignature250");
    definePointCloudBuffers<PPFSignature>(pc, "PPFSignature");
    definePointCloudBuffers<CPPFSignature>(pc, "CPPFSignature");
    definePointCloudBuffers<PPFRGBSignature>(pc, "PPFRGBSignature");
    definePointCloudBuffers<NormalBasedSignature12>(pc, "NormalBasedSignature12");
    definePointCloudBuffers<FPFHSignature33>(pc, "FPFHSignature33");
    definePointCloudBuffers<VFHSignature308>(pc, "VFHSignature308");
    definePointCloudBuffers<GRSDSignature21>(pc, "GRSDSignature21");
    definePointCloudBuffers<ESFSignature640>(pc, "ESFSignature640");
    definePointCloudBuffers<BRISKSignature512>(pc, "BRISKSignature512");
    definePointCloudBuffers<Narf36>(pc, "Narf36");
    definePointCloudBuffers<IntensityGradient>(pc, "IntensityGradient");
    definePointCloudBuffers<PointWithScale>(pc, "PointWithScale");
    definePointCloudBuffersRGB<PointSurfel>(pc, "PointSurfel");
    definePointCloudBuffers<ShapeContext1980>(pc, "ShapeContext1980");
    definePointCloudBuffers<UniqueShapeContext1960>(pc, "UniqueShapeContext1960");
    definePointCloudBuffers<SHOT352>(pc, "SHOT352");
    definePointCloudBuffers<SHOT1344>(pc, "SHOT1344");
    definePointCloudBuffers<PointUV>(pc, "PointUV");
    definePointCloudBuffers<ReferenceFrame>(pc, "ReferenceFrame");
    definePointCloudBuffers<PointDEM>(pc, "PointDEM");
}
