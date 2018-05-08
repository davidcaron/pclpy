
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/common/norms.h>



template <typename FloatVectorT>
void defineCommonNormsFunctions1(py::module &m) {
    m.def("B_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::B_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("CS_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::CS_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("Div_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::Div_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("HIK_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::HIK_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("JM_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::JM_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("KL_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::KL_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("K_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int, float, float> (&pcl::K_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a, "P1"_a, "P2"_a);
    m.def("L1_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::L1_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("L2_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::L2_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("L2_Norm_SQR", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::L2_Norm_SQR<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("Linf_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::Linf_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("PF_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int, float, float> (&pcl::PF_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a, "P1"_a, "P2"_a);
    m.def("Sublinear_Norm", py::overload_cast<FloatVectorT, FloatVectorT, int> (&pcl::Sublinear_Norm<FloatVectorT>), "A"_a, "B"_a, "dim"_a);
    m.def("selectNorm", py::overload_cast<FloatVectorT, FloatVectorT, int, pcl::NormType> (&pcl::selectNorm<FloatVectorT>), "A"_a, "B"_a, "dim"_a, "norm_type"_a);
}

void defineCommonNormsFunctions(py::module &m) {
    defineCommonNormsFunctions1<std::vector<float>>(m);
}

void defineCommonNormsClasses(py::module &sub_module) {
    defineCommonNormsFunctions(sub_module);
    py::enum_<pcl::NormType>(sub_module, "NormType")
        .value("L1", pcl::NormType::L1)
        .value("L2_SQR", pcl::NormType::L2_SQR)
        .value("L2", pcl::NormType::L2)
        .value("LINF", pcl::NormType::LINF)
        .value("JM", pcl::NormType::JM)
        .value("B", pcl::NormType::B)
        .value("SUBLINEAR", pcl::NormType::SUBLINEAR)
        .value("CS", pcl::NormType::CS)
        .value("DIV", pcl::NormType::DIV)
        .value("PF", pcl::NormType::PF)
        .value("K", pcl::NormType::K)
        .value("KL", pcl::NormType::KL)
        .value("HIK", pcl::NormType::HIK)
        .export_values();
}