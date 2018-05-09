
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineCommon_kissFftGutsClasses(py::module &);
void defineCommonAnglesClasses(py::module &);
void defineCommonBivariatePolynomialClasses(py::module &);
void defineCommonBoostClasses(py::module &);
void defineCommonCentroidClasses(py::module &);
void defineCommonColorsClasses(py::module &);
void defineCommonCommonClasses(py::module &);
void defineCommonConcatenateClasses(py::module &);
void defineCommonCopyPointClasses(py::module &);
void defineCommonDistancesClasses(py::module &);
void defineCommonFeatureHistogramClasses(py::module &);
void defineCommonFileIoClasses(py::module &);
void defineCommonGeometryClasses(py::module &);
void defineCommonIntensityClasses(py::module &);
void defineCommonIntersectionsClasses(py::module &);
void defineCommonIoClasses(py::module &);
void defineCommonKissFftClasses(py::module &);
void defineCommonKissFftrClasses(py::module &);
void defineCommonNormsClasses(py::module &);
void defineCommonPcaClasses(py::module &);
void defineCommonPiecewiseLinearFunctionClasses(py::module &);
void defineCommonPointOperatorsClasses(py::module &);
void defineCommonPointTestsClasses(py::module &);
void defineCommonPolynomialCalculationsClasses(py::module &);
void defineCommonPosesFromMatchesClasses(py::module &);
void defineCommonProjectionMatrixClasses(py::module &);
void defineCommonRandomClasses(py::module &);
void defineCommonGenerateClasses(py::module &);
void defineCommonSpringClasses(py::module &);
void defineCommonTimeClasses(py::module &);
void defineCommonCommonHeadersClasses(py::module &);
void defineCommonTransformationFromCorrespondencesClasses(py::module &);
void defineCommonTransformsClasses(py::module &);
void defineCommonUtilsClasses(py::module &);
void defineCommonVectorAverageClasses(py::module &);


void defineCommonClasses(py::module &m) {
    py::module m_common = m.def_submodule("common", "Submodule common");
    defineCommon_kissFftGutsClasses(m_common);
    defineCommonAnglesClasses(m_common);
    defineCommonBivariatePolynomialClasses(m_common);
    defineCommonBoostClasses(m_common);
    defineCommonCentroidClasses(m_common);
    defineCommonColorsClasses(m_common);
    defineCommonCommonClasses(m_common);
    defineCommonConcatenateClasses(m_common);
    defineCommonCopyPointClasses(m_common);
    defineCommonDistancesClasses(m_common);
    defineCommonFeatureHistogramClasses(m_common);
    defineCommonFileIoClasses(m_common);
    defineCommonGeometryClasses(m_common);
    defineCommonIntensityClasses(m_common);
    defineCommonIntersectionsClasses(m_common);
    defineCommonIoClasses(m_common);
    defineCommonKissFftClasses(m_common);
    defineCommonKissFftrClasses(m_common);
    defineCommonNormsClasses(m_common);
    defineCommonPcaClasses(m_common);
    defineCommonPiecewiseLinearFunctionClasses(m_common);
    defineCommonPointOperatorsClasses(m_common);
    defineCommonPointTestsClasses(m_common);
    defineCommonPolynomialCalculationsClasses(m_common);
    defineCommonPosesFromMatchesClasses(m_common);
    defineCommonProjectionMatrixClasses(m_common);
    defineCommonRandomClasses(m_common);
    defineCommonGenerateClasses(m_common);
    defineCommonSpringClasses(m_common);
    defineCommonTimeClasses(m_common);
    defineCommonCommonHeadersClasses(m_common);
    defineCommonTransformationFromCorrespondencesClasses(m_common);
    defineCommonTransformsClasses(m_common);
    defineCommonUtilsClasses(m_common);
    defineCommonVectorAverageClasses(m_common);
}