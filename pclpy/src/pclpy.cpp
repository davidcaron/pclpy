
#pragma warning (disable : 4367)
#pragma warning (disable : 4267)

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <pcl/point_representation.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);

#include "make_opaque_vectors.hpp"
#include "eigen_bind.hpp"
#include "generated_modules/__main_loader.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

void definePointClouds(py::module &);
void defineVectorClasses(py::module &);
void definePointTypes(py::module &);

PYBIND11_MODULE(pcl, m) {
    m.doc() = "PCL python bindings";

    definePointTypes(m);

    py::module m_vector = m.def_submodule("vectors", "Submodule for vectors");
    defineEigenClasses(m_vector);
    defineVectorClasses(m_vector);

    defineClasses(m);

    py::module pc = m.attr("PointCloud");
    definePointClouds(pc);

}
