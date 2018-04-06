
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"
#include "recognition/auxiliary.hpp"
#include "recognition/boost.hpp"
#include "recognition/bvh.hpp"
#include "recognition/distance_map.hpp"
#include "recognition/hypothesis.hpp"
#include "recognition/mask_map.hpp"
#include "recognition/model_library.hpp"
#include "recognition/obj_rec_ransac.hpp"
#include "recognition/orr_graph.hpp"
#include "recognition/orr_octree.hpp"
#include "recognition/orr_octree_zprojection.hpp"
#include "recognition/point_types.hpp"
#include "recognition/quantized_map.hpp"
#include "recognition/region_xy.hpp"
#include "recognition/dense_quantized_multi_mod_template.hpp"
#include "recognition/dot_modality.hpp"
#include "recognition/color_gradient_dot_modality.hpp"
#include "recognition/dotmod.hpp"
#include "recognition/simple_octree.hpp"
#include "recognition/rigid_transform_space.hpp"
#include "recognition/sparse_quantized_multi_mod_template.hpp"
#include "recognition/quantizable_modality.hpp"
#include "recognition/color_gradient_modality.hpp"
#include "recognition/color_modality.hpp"
#include "recognition/linemod.hpp"
#include "recognition/voxel_structure.hpp"
#include "recognition/crh_alignment.hpp"
#include "recognition/implicit_shape_model.hpp"
#include "recognition/surface_normal_modality.hpp"
#include "recognition/line_rgbd.hpp"


void defineRecognitionClasses(py::module &m) {
    py::module m_recognition = m.def_submodule("recognition", "Submodule recognition");
    defineRecognitionAuxiliaryClasses(m_recognition);
    defineRecognitionBoostClasses(m_recognition);
    defineRecognitionBvhClasses(m_recognition);
    defineRecognitionDistanceMapClasses(m_recognition);
    defineRecognitionHypothesisClasses(m_recognition);
    defineRecognitionMaskMapClasses(m_recognition);
    defineRecognitionModelLibraryClasses(m_recognition);
    defineRecognitionObjRecRansacClasses(m_recognition);
    defineRecognitionOrrGraphClasses(m_recognition);
    defineRecognitionOrrOctreeClasses(m_recognition);
    defineRecognitionOrrOctreeZprojectionClasses(m_recognition);
    defineRecognitionPointTypesClasses(m_recognition);
    defineRecognitionQuantizedMapClasses(m_recognition);
    defineRecognitionRegionXyClasses(m_recognition);
    defineRecognitionDenseQuantizedMultiModTemplateClasses(m_recognition);
    defineRecognitionDotModalityClasses(m_recognition);
    defineRecognitionColorGradientDotModalityClasses(m_recognition);
    defineRecognitionDotmodClasses(m_recognition);
    defineRecognitionSimpleOctreeClasses(m_recognition);
    defineRecognitionRigidTransformSpaceClasses(m_recognition);
    defineRecognitionSparseQuantizedMultiModTemplateClasses(m_recognition);
    defineRecognitionQuantizableModalityClasses(m_recognition);
    defineRecognitionColorGradientModalityClasses(m_recognition);
    defineRecognitionColorModalityClasses(m_recognition);
    defineRecognitionLinemodClasses(m_recognition);
    defineRecognitionVoxelStructureClasses(m_recognition);
    defineRecognitionCrhAlignmentClasses(m_recognition);
    defineRecognitionImplicitShapeModelClasses(m_recognition);
    defineRecognitionSurfaceNormalModalityClasses(m_recognition);
    defineRecognitionLineRgbdClasses(m_recognition);
}