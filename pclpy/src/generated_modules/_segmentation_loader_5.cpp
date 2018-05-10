
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "segmentation/region_growing.hpp"
#include "segmentation/sac_segmentation.hpp"


void defineSegmentationClasses5(py::module &m) {
    py::module m_segmentation = m.def_submodule("segmentation", "Submodule segmentation");
    defineSegmentationRegionGrowingClasses(m_segmentation);
    defineSegmentationSacSegmentationClasses(m_segmentation);
}