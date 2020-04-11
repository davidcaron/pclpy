import os
from os.path import join
import platform
import sys

from pkgconfig_utils import get_include_dir

INDENT = " " * 4
BASE_SUB_MODULE_NAME = "sub_module"

PATH_SRC = join("..", "pclpy", "src")
PATH_MAIN_CPP = join(PATH_SRC, "pclpy.cpp")
PATH_MODULES = join(PATH_SRC, "generated_modules")
PATH_LOADER = join(PATH_MODULES, "__main_loader.hpp")

CONDA = 'conda' in sys.version or os.path.exists(join(sys.prefix, 'conda-meta'))

if platform.system() == "Windows":
    if CONDA:
        PCL_BASE = join(sys.prefix, "Library", "include", "pcl-1.9", "pcl")
    else:
        PCL_BASE = join(os.environ["PCL_ROOT"], "include", "pcl-1.8", "pcl")
elif CONDA:
    PCL_BASE = join(sys.prefix, "include", "pcl-1.9", "pcl")
else:
    PCL_BASE = join(get_include_dir(), "pcl")

common_includes = """
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;
"""

cpp_header = """
PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "make_opaque_vectors.hpp"
"""

# ----------------------
# which modules to build
# ----------------------

MODULES_TO_BUILD = [
    '2d',
    'common',
    'features',
    'geometry',
    'filters',
    'io',
    'kdtree',
    'keypoints',
    'octree',
    'recognition',
    'sample_consensus',
    'search',
    'segmentation',
    'stereo',
    'surface',
    'tracking',
    # 'visualization',
]
# skipped for now:
# , 'ml', 'people', 'outofcore', 'registration']

# -----------------------------
# specific generator parameters
# -----------------------------

IGNORE_INHERITED_INSTANTIATIONS = [
    # "class"
    "OrganizedEdgeBase",
    "OrganizedEdgeFromRGB",
]

INHERITED_ENUMS = [
    # ("class", "type")
    ("ImageRGB24", "Timestamp"),
    ("ImageYUV422", "Timestamp"),
]

# ---
# When a base class contains only some types of the child class, and they are unordered.
# This happens but is sort of rare in PCL.

INHERITED_TEMPLATED_TYPES_FILTER = {
    # ("base_class", "child_class"): [0, 3, 5]

    "Feature": [0, 2],  # this is not a mistake, but it should be fixed. Feature is inherited from a lot.
    # I think narf or narf_descriptor could be an exception

    ("UniqueShapeContext", "FeatureWithLocalReferenceFrames"): [0, 2],
    ("SHOTEstimationBase", "FeatureWithLocalReferenceFrames"): [0, 3],
    ("SHOTEstimation", "FeatureWithLocalReferenceFrames"): [0, 3],
    ("SHOTEstimationOMP", "FeatureWithLocalReferenceFrames"): [0, 3],
    ("SHOTColorEstimation", "FeatureWithLocalReferenceFrames"): [0, 3],
    ("SHOTColorEstimationOMP", "FeatureWithLocalReferenceFrames"): [0, 3],
}

KEEP_ASIS_TYPES = {
    "Eigen::",
    "pcl::",
    "std::",
    "boost::shared_ptr",
    "boost::filesystem",
    "boost::uint64_t",
    "uint8_t",
    "unsigned",
    "unsigned char",
    "uint16_t",
    "uint32_t",
    "uint64_t",
    "int",
    "int8_t",
    "int16_t",
    "int32_t",
    "int64_t",
    "bool",
    "void",
    "char",
    "float",
    "double",
    "size_t",
    "off_t",
}

# explicitely excluded classes
CLASSES_TO_IGNORE = [
    # ("module", "header", "class")

    # (not implemented in pcl source code)
    ("outofcore", "outofcore_iterator_base.h", "OutofcoreBreadthFirstIterator"),
    ("outofcore", "outofcore_iterator_base.h", "OutofcoreLeafIterator"),
    ("common", "colors.h", "ColorLUT"),  # ColorLUTName tamplate type
    # constructor seems to access private member...
    ("io", "obj_io.h", "MTLReader"),
    ("filters", "voxel_grid_label.h", "VoxelGridLabel"),  # needs PointXYZRGBL which is skipped
    ("stereo", "digital_elevation_map.h", "DigitalElevationMapBuilder"),  # needs PointDEM which is skipped
    ("io", "io_exception.h", "IOException"),  # linking error
    ("visualization", "interactor_style.h", "PCLHistogramVisualizerInteractorStyle"),  # linking error
]

CUSTOM_OVERLOAD_TYPES = {
    # ("class", "type"): "type_replacement"
    ("FastBilateralFilter", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("MedianFilter", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("FrustrumCulling", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("VoxelGridOcclusionEstimation", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("ConditionalRemoval", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("FastBilateralFilterOMP", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("CropBox", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("VoxelGridCovariance", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("ApproximateVoxelGrid", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("ProjectInliers", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("CropHull", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("BilateralFilter", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("UniformSampling", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("VoxelGrid", "PointCloud"): "pcl::Filter<PointT>::PointCloud",
    ("SamplingSurfaceNormal", "PointCloud"): "pcl::Filter<PointT>::PointCloud",

    ("DepthImage", "FrameWrapper::Ptr"): "FrameWrapper::Ptr",
    ("IRImage", "FrameWrapper::Ptr"): "FrameWrapper::Ptr",

    ("PointCloudColorHandlerCustom",
     "PointCloudConstPtr"): "PointCloudColorHandler<PointT>::PointCloud::ConstPtr",
    ("PointCloudColorHandlerGenericField",
     "PointCloudConstPtr"): "PointCloudColorHandler<PointT>::PointCloud::ConstPtr",
    ("PointCloudColorHandlerHSVField",
     "PointCloudConstPtr"): "PointCloudColorHandler<PointT>::PointCloud::ConstPtr",
    ("PointCloudColorHandlerLabelField",
     "PointCloudConstPtr"): "PointCloudColorHandler<PointT>::PointCloud::ConstPtr",
    ("PointCloudColorHandlerRGBAField",
     "PointCloudConstPtr"): "PointCloudColorHandler<PointT>::PointCloud::ConstPtr",
    ("PointCloudColorHandlerRGBField",
     "PointCloudConstPtr"): "PointCloudColorHandler<PointT>::PointCloud::ConstPtr",
    ("PointCloudColorHandlerRandom",
     "PointCloudConstPtr"): "PointCloudColorHandler<PointT>::PointCloud::ConstPtr",

    ("LeastMedianSquares", "SampleConsensusModelPtr"): "pcl::SampleConsensusModel<PointT>::Ptr",
    ("MaximumLikelihoodSampleConsensus", "SampleConsensusModelPtr"): "pcl::SampleConsensusModel<PointT>::Ptr",
    ("MEstimatorSampleConsensus", "SampleConsensusModelPtr"): "pcl::SampleConsensusModel<PointT>::Ptr",
    ("ProgressiveSampleConsensus", "SampleConsensusModelPtr"): "pcl::SampleConsensusModel<PointT>::Ptr",
    ("RandomSampleConsensus", "SampleConsensusModelPtr"): "pcl::SampleConsensusModel<PointT>::Ptr",
    ("RandomizedMEstimatorSampleConsensus", "SampleConsensusModelPtr"): "pcl::SampleConsensusModel<PointT>::Ptr",
    ("RandomizedRandomSampleConsensus", "SampleConsensusModelPtr"): "pcl::SampleConsensusModel<PointT>::Ptr",

    ("DisparityMapConverter", "PointCloud"): "pcl::PointCloud<PointT>",
    ("PointCloud", "PointCloud<PointT>"): "pcl::PointCloud<PointT>",
}

# types that are explicitely considered as part of the "pcl" namespace
GLOBAL_PCL_IMPORTS = [
    "IndicesPtr",
    "IndicesConstPtr",
    "PointIndicesConstPtr",
    "Correspondence",
    "PointIndices",
    "ModelCoefficients",
    "PointWithRange",
    "PCLBase",
    "PointCloud<PointT>",
    "PlanarRegion",
    "PointXYZ",
    "SVMData",
    "InterpolationType",  # local enum
    "NormType",  # local enum
    "ReferenceFrame",
    "GASDSignature512",
    "GASDSignature984",
]

EXPLICIT_IMPORTED_TYPES = [
    "Camera",
    "PointCloudGeometryHandler",
    "PointCloudColorHandler",
]

EXTERNAL_INHERITANCE = [
    "svm_parameter",
    "svm_model",
    "std",
    "boost",
    "vtk",
]

SKIPPED_INHERITANCE = [
    "boost::",
    "vtk",
]

TEMPLATED_METHOD_TYPES = {
    "PointT": "PCL_POINT_TYPES",
    "Point": "PCL_POINT_TYPES",
    "PointInT": "PCL_POINT_TYPES",
    "PointLT": ["pcl::Label"],
    "PointOutT": "PCL_POINT_TYPES",
    # PointTDiff is used in kdtree and search to query using different point type
    # we try to limit the number of points to reduce ram issues during build
    "PointTDiff": ["pcl::PointXYZ"],
    "PointRFT": ["pcl::ReferenceFrame"],
    "StateT": "PCL_STATE_POINT_TYPES",
    "OutputType": "PCL_POINT_TYPES",
    "PointSource": "PCL_XYZ_POINT_TYPES",
    "PointFeature": "PCL_FEATURE_POINT_TYPES",
    "T": "PCL_POINT_TYPES",
    "PointNT": "PCL_NORMAL_POINT_TYPES",
    "NormalT": "PCL_NORMAL_POINT_TYPES",
    "Normal": "PCL_NORMAL_POINT_TYPES",
    "GradientT": ["pcl::IntensityGradient"],
    "P1": "PCL_XYZ_POINT_TYPES",
    "PointType1": "PCL_POINT_TYPES",
    "Point1T": "PCL_POINT_TYPES",
    "PointIn1T": "PCL_POINT_TYPES",
    "PointIn2T": "PCL_POINT_TYPES",
    "P2": "PCL_XYZ_POINT_TYPES",
    "PointType2": "PCL_POINT_TYPES",
    "Point2T": "PCL_POINT_TYPES",
    "FeatureT": "PCL_FEATURE_POINT_TYPES",
    "Scalar": ["float"],
    "CloudT": "PCL_POINT_CLOUD_TYPES",
    "real": ["float", "double"],
    "FloatVectorT": ["std::vector<float>"],
    "ValT": ["float", "uint8_t", "uint32_t"],
    "MeshT": ["pcl::geometry::PolygonMesh", "pcl::geometry::QuadMesh", "pcl::geometry::TriangleMesh"],
    "HalfEdgeMeshT": ["pcl::geometry::PolygonMesh", "pcl::geometry::QuadMesh", "pcl::geometry::TriangleMesh"],
    "MeshTraitsT": [],  # nothing for now

    # Eigen::MatrixBase<Derived>
    "Derived": ["float", "double"],
    "OtherDerived": ["float", "double"],
}

pcl_visualizer_xyz = ["pcl::PointSurfel", "pcl::PointXYZ", "pcl::PointXYZL", "pcl::PointXYZI", "pcl::PointXYZRGB",
                      "pcl::PointXYZRGBA", "pcl::PointNormal", "pcl::PointXYZRGBNormal", "pcl::PointXYZRGBL",
                      "pcl::PointWithRange"]

SPECIFIC_TEMPLATED_METHOD_TYPES = {
    # ("class_name", "method_name", ("templated_parameter_names", ))
    # ("header_name", "", ("templated_parameter_names", ))
    # if method_name is empty, it's considered as the default template type for this class
    ("ImageViewer", "", ("T",)): ("PCL_RGB_POINT_TYPES",),
    ("ImageViewer", "", ("PointT",)): ("PCL_RGB_POINT_TYPES",),
    ("ImageViewer", "addRectangle", ("T",)): ("PCL_XYZ_POINT_TYPES",),

    ("LZFBayer8ImageReader", "", ("PointT",)): ("PCL_RGB_POINT_TYPES",),
    ("LZFDepth16ImageReader", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("LZFRGB24ImageReader", "", ("PointT",)): ("PCL_RGB_POINT_TYPES",),
    ("LZFYUV422ImageReader", "", ("PointT",)): ("PCL_RGB_POINT_TYPES",),

    ("centroid.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("centroid.h", "", ("PointT", "Scalar")): ("PCL_XYZ_POINT_TYPES", ["float"]),
    ("centroid.h", "", ("PointInT", "PointOutT")): ("PCL_XYZ_POINT_TYPES", "PCL_XYZ_POINT_TYPES"),
    # compromise for weird compile errors... ex: PointXYZINormal and PointXYZI don't work...
    ("centroid.h", "computeCentroid", ("PointInT", "PointOutT")): (["pcl::PointXYZ", "pcl::PointXYZRGBA"],
                                                                   ["pcl::PointXYZ", "pcl::PointXYZRGBA"]),

    ("shapes.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),

    ("common.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("filter_indices.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("filter.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("filter.h", "removeNaNNormalsFromPointCloud", ("PointT",)): ("PCL_NORMAL_POINT_TYPES",),
    ("morphological_filter.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("segment_differences.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("distances.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("distances.h", "", ("PointType1", "PointType2")): ("PCL_XYZ_POINT_TYPES", "PCL_XYZ_POINT_TYPES"),
    ("projection_matrix.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("point_tests.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),

    ("from_meshes.h", "", ("PointT", "PointNT")): ("PCL_XYZ_POINT_TYPES", "PCL_NORMAL_POINT_TYPES"),

    ("transforms.h", "", ("PointT",)): ("PCL_XYZ_POINT_TYPES",),
    ("transforms.h", "", ("PointT", "Scalar")): ("PCL_XYZ_POINT_TYPES", ["float"]),
    ("transforms.h", "transformPointWithNormal", ("PointT",)): ("PCL_NORMAL_POINT_TYPES",),
    ("transforms.h", "transformPointCloudWithNormals", ("PointT",)): ("PCL_NORMAL_POINT_TYPES",),
    ("transforms.h", "transformPointCloudWithNormals", ("PointT", "Scalar")): ("PCL_NORMAL_POINT_TYPES", ["float"]),

    ("MLSResult", "computeMLSSurface", ("PointT",)): ("PCL_XYZ_POINT_TYPES", ),

    # (ShapeContext1980, UniqueShapeContext1960) and (SHOT352, SHOT1344) have fields of the same name
    # this is a workaround
    ("copy_point.h", "", ("PointInT", "PointOutT")): ("PCL_XYZ_POINT_TYPES", "PCL_XYZ_POINT_TYPES"),

    ("Camera", "", ("PointT",)): (pcl_visualizer_xyz,),
    ("PCLVisualizer", "", ("PointT",)): (pcl_visualizer_xyz,),
    ("PCLVisualizer", "", ("PointT", "GradientT")): (pcl_visualizer_xyz, ["pcl::IntensityGradient"]),
    ("PCLVisualizer", "", ("PointNT",)): (["pcl::PointNormal", "pcl::PointXYZRGBNormal", "pcl::PointXYZINormal",
                                           "pcl::PointXYZLNormal", "pcl::PointSurfel"],),
    ("PCLVisualizer", "", ("PointT", "PointNT")): (pcl_visualizer_xyz, "PCL_NORMAL_POINT_TYPES"),
}

EXPLICIT_INCLUDES = {
    # (module, header_name): "#include...",
    ("geometry", "mesh_io.h"): ("#include <pcl/geometry/polygon_mesh.h>\n"
                                "#include <pcl/geometry/triangle_mesh.h>"),
    ("geometry", "get_boundary.h"): ("#include <pcl/geometry/polygon_mesh.h>\n"
                                     "#include <pcl/geometry/quad_mesh.h>\n"
                                     "#include <pcl/geometry/triangle_mesh.h>"),
    ("geometry", "mesh_conversion.h"): ("#include <pcl/geometry/polygon_mesh.h>\n"
                                        "#include <pcl/geometry/quad_mesh.h>\n"
                                        "#include <pcl/geometry/triangle_mesh.h>"),
    ("segmentation", "plane_refinement_comparator.h"): "#include <pcl/ModelCoefficients.h>",
    ("features", "narf_descriptor.h"): "#include <pcl/range_image/range_image.h>",
    ("features", "from_meshes.h"): "#include <pcl/Vertices.h>",
    ("common", "synchronizer.h"): '#include <boost/thread/mutex.hpp>',
    ("visualization", "interactor_style.h"): "#include <pybind11/functional.h>",
    ("visualization", "image_viewer.h"): "#include <pybind11/functional.h>",
    ("visualization", "pcl_visualizer.h"): "#include <pybind11/functional.h>\n"
                                           '#include "PyVTKObject.h"\n'
                                           "#pragma warning(disable : 4996)",
    ("recognition", "orr_octree.h"): "#pragma warning(disable : 4800)",
    ("recognition", "obj_rec_ransac.h"): "#pragma warning(disable : 4267)",
    ("recognition", "model_library.h"): "#pragma warning(disable : 4267)",
    ("recognition", "implicit_shape_model.h"): "#pragma warning(disable : 4267)",
    ("sample_consensus", "model_types.h"): "#pragma warning(disable : 4996)",
    ("surface", "concave_hull.h"): "#pragma warning(disable : 4996)",
    ("features", "grsd.h"): "#pragma warning(disable : 4506)",
    ("outofcore", "axes.h"): "#include <vtkPointData.h>",
    ("octree", "octree_base.h"): "#include <pcl/octree/octree_pointcloud_voxelcentroid.h>\n"
                                 "#include <pcl/octree/octree_pointcloud_singlepoint.h>",
    ("octree", "octree_pointcloud.h"): "#include <pcl/octree/octree_pointcloud_voxelcentroid.h>\n"
                                       "#include <pcl/octree/octree_pointcloud_singlepoint.h>",
}

DONT_HOLD_WITH_BOOST_SHARED_PTR = [
    "PCLVisualizerInteractorStyle",
]

EXTRA_FUNCTIONS = {
    "PCLVisualizer":
        """
        //Extra functions
        cls.def("getRenderWindow", [] (Class &v) {ob}
            PyTypeObject *type_ = py::module::import("vtk").attr("vtkRenderWindow")().ptr()->ob_type;
            PyObject *win = PyVTKObject_FromPointer(type_, nullptr, v.getRenderWindow().Get());
            py::object win_obj = py::reinterpret_borrow<py::object>(win);
            return win_obj;
        {cb});
        cls.def("setupQWidget", [] (Class &v, py::object &qvtk_widget) {ob}
            py::object qvtk_interactor = qvtk_widget.attr("_Iren");
            vtkRenderWindowInteractor *vtk_interactor = static_cast<vtkRenderWindowInteractor*>(PyVTKObject_GetObject(qvtk_interactor.ptr()));
            v.setupInteractor(vtk_interactor, v.getRenderWindow().Get());
        {cb});"""
    ,
}

# ------------
# what to skip
# ------------

HEADERS_TO_SKIP = [
    # ("module", "header")
    ("io", "pxc_grabber.h"),  # deprecated
    ("io", "dinast_grabber.h"),
    ("", "sse.h"),  # don't need that
    ("", "point_representation.h"),  # I could be wrong, but this seems covered in python with the buffer protocol..
    ("", "conversions.h"),  # can't find overloaded functions
    ("ml", "multi_channel_2d_comparison_feature_handler.h"),  # can't find class FeatureHandlerCodeGenerator ??
    ("", "pcl_tests.h"),
    ("", "for_each_type.h"),

    ("2d", "kernel.h"),  # missing impl/kernel.hpp in Windows release
    ("2d", "edge.h"),  # missing impl/kernel.hpp in Windows release

    ("io", "hdl_grabber.h"),
    ("io", "vlp_grabber.h"),
    ("io", "openni.h"),
    ("io", "openni2_grabber.h"),
    ("io", "openni2_convert.h"),
    ("io", "openni2_device.h"),
    ("io", "openni2_device_info.h"),
    ("io", "openni2_device_manager.h"),
    ("io", "openni2_frame_listener.h"),
    ("io", "openni2_metadata_wrapper.h"),
    ("io", "openni2_timer_filter.h"),
    ("io", "openni2_video_mode.h"),

    ("surface", "multi_grid_octree_data.h"),  # compile error in PCL "OctNode is not a member of pcl::poisson"

    ("geometry", "get_boundary.h"),  # some template types need more investigation
    ("geometry", "mesh_conversion.h"),  # some template types need more investigation
    ("io", "vtk_lib_io.h"),  # functions overload somehow hard to match with VTK

    ("recognition", "hv_go.h"),  # depends on metslib

    ("", "exceptions.h"),  # todo: implement exceptions
    ("registration", "exceptions.h"),  # todo: implement exceptions
    ("segmentation", "conditional_euclidean_clustering.h"),  # setConditionFunction hard to implement...
    ("segmentation", "seeded_hue_segmentation.h"),  # not exported in dll for some reason. Linking error.
    ("segmentation", "euclidean_cluster_comparator.h"),
    ("common", "time_trigger.h"),  # init containing boost::function
    ("common", "synchronizer.h"),
    ("common", "spring.h"),  # not compiled in Windows PCL release

    ("visualization", "pcl_painter2D.h"),  # tricky protected vtkContextItem destructor
    ("visualization", "pcl_context_item.h"),  # tricky protected vtkContextItem destructor
    ("visualization", "interactor.h"),  # I think CppHeaderParser chokes because there are too many macros
    ("visualization", "histogram_visualizer.h"),  # link error (visualization::RenWinInteract::RenWinInteract(void))
    ("visualization", "simple_buffer_visualizer.h"),  # link error (visualization::RenWinInteract::RenWinInteract(void))
    ("visualization", "ren_win_interact_map.h"),  # link error (visualization::RenWinInteract::RenWinInteract(void))

    ("common", "gaussian.h"),  # templated method?
    ("common", "eigen.h"),  # too many templated functions, skip for now

    ("recognition", "trimmed_icp.h"),  # depends on registration
    ("recognition", "obj_rec_ransac.h"),  # depends on trimmed_icp, which depends on registration

    ("keypoints", "smoothed_surfaces_keypoint.h"),
    # Inherits from Keypoint <PointT, PointT> (which seems weird to me...)

    ("features", "range_image_border_extractor.h"),  # depends on range_image
    # ImportError: generic_type: type "NarfDescriptor" referenced unknown
    # base type "pcl::Feature<pcl::PointWithRange,pcl::Narf36>"
    ("features", "narf_descriptor.h"),  # depends on range_image
    ("features", "narf.h"),  # depends on range_image
    ("keypoints", "narf_keypoint.h"),  # depends on range_image and range_image_border_extractor
    ("features", "ppfrgb.h"),  # linking error on linux

    ("segmentation", "random_walker.h"),  # not familiar enough to know how to include, skip for now

    ("filters", "conditional_removal.h"),
    # todo: parser error for ConditionalRemoval (int extract_removed_indices = false) :
    ("filters", "model_outlier_removal.h"),  # todo: boost::function as parameter
]

FUNCTIONS_TO_SKIP = [
    # ("header_name", "function_name")
    ("file_io.h", "isValueFinite"),
    ("file_io.h", "copyValueString"),
    ("file_io.h", "copyStringValue"),
    ("png_io.h", "PCL_DEPRECATED"),
    ("png_io.h", "savePNGFile"),  # no matching overloaded function found
    ("correspondence.h", "getRejectedQueryIndices"),
    ("point_traits.h", "getFieldValue"),
    ("point_traits.h", "setFieldValue"),
    ("rsd.h", "getFeaturePointCloud"),  # template <int N>
    ("io.h", "concatenateFields"),  # way too many template combinations... skip for now
    ("io.h", "isSamePointType"),  # way too many template combinations... skip for now
    ("common.h", "getMinMax"),  # only for point histogram type
    ("file_io.h", "getAllPcdFilesInDirectory"),  # linking error
    ("io.h", "getFieldsSizes"),  # linking error
    ("io_exception.h", "throwIOException"),  # no matching overloaded function found
    ("filter.h", "removeNaNNormalsFromPointCloud"),  # (fixable) PointT XYZ and Normal point types for 2 functions
    ("transforms.h", "transformPointCloudWithNormals"),  # (fixable) PointT XYZ and Normal point types for 2 functions
    ("transforms.h", "transformPointWithNormal"),  # (fixable) PointT XYZ and Normal point types for 2 functions
    ("intersections.h", "planeWithPlaneIntersection"),  # couldn't deduce template parameter ‘Return’
    ("intersections.h", "threePlanesIntersection"),  # couldn't deduce template parameter ‘Return’

    # todo: I think most of these could be removed. They were added before I realized there was a bug.
    ("io.h", "copyPointCloud"),  # no matching overload found...
    ("io.h", "getFieldIndex"),  # no matching overload found...
    ("io.h", "getApproximateIndices"),  # kdtree/io.h no matching overload found...
    ("voxel_grid.h", "getMinMax3D"),  # no matching overload found...
    ("sac_model_plane.h", "pointToPlaneDistance"),  # no matching overload found...
    ("sac_model_plane.h", "pointToPlaneDistanceSigned"),  # no matching overload found...
    ("sac_model_plane.h", "projectPoint"),  # no matching overload found...
    ("region_xy.h", "read"),  # is this function useful? "Type" template could be anything?
    ("region_xy.h", "write"),  # is this function useful? "Type" template could be anything?
    ("extract_clusters.h", "extractEuclideanClusters"),  # skip for now, use EuclideanClusterExtraction instead
    ("extract_labeled_clusters.h", "extractLabeledEuclideanClusters"),
    # skip for now, use EuclideanClusterExtraction instead
    ("extract_polygonal_prism_data.h", "isXYPointIn2DXYPolygon"),  # unknown type error...
    ("extract_polygonal_prism_data.h", "isPointIn2DPolygon"),  # unknown type error...
    ("polygon_operations.h", "approximatePolygon"),  # cound not deduce return type
    ("polygon_operations.h", "approximatePolygon2D"),  # no matching overload found...
    # ("filter.h", "removeNaNFromPointCloud"),  # no matching overload found...
    # ("filter.h", "removeNaNNormalsFromPointCloud"),  # no matching overload found...
    ("normal_3d.h", "computePointNormal"),  # no matching overload found...
    ("normal_3d.h", "flipNormalTowardsViewpoint"),  # no matching overload found...
]

SPECIALIZED_TEMPLATED_TYPES_TO_SKIP = [
    "pcl::PCLPointCloud2",  # don't implement this for now
]

SUBMODULES_TO_SKIP = [
    "opennurbs",
    "face_detection",  # depends on ml/decision_tree_data_provider
    "metslib",  # lots of warnings, skip for now...
]

ATTRIBUTES_TO_SKIP = {
    # ("module", "header", "class"): ["attr1", "attr2"]
    ("features", "shot.h", "SHOTColorEstimation"): ["sRGB_LUT", "sXYZ_LUT"],  # todo: linking error
    ("features", "narf.h", "Narf"): ["max_no_of_threads"],  # todo: linking error
    ("features", "organized_edge_detection.h", "OrganizedEdgeBase"): ["num_of_edgetype_"],  # linking error
    ("octree", "octree_key.h", "OctreeKey"): ["maxDepth"],  # linking error
    ("recognition", "orr_octree.h", "ORROctree"): ["createLeaf"],  # todo: linking error
}

METHODS_TO_SKIP = [
    # ("class", "method")

    ("PointCloud", "insert"),  # templated InputIterator

    ("ASCIIReader", "setInputFields"),
    ("ASCIIReader", "read"),  # PCLPointCloud2 not implemented for now
    ("PCLPlotter", "addPlotData"),
    ("PCLPlotter", "addFeatureHistogram"),
    ("PCLHistogramVisualizer", "spinOnce"),
    ("PCLHistogramVisualizer", "addFeatureHistogram"),
    ("PCLHistogramVisualizer", "updateFeatureHistogram"),
    ("ORROctree", "createLeaf"),  # linking error
    ("PCLVisualizer", "getRenderWindow"),  # wrapped to return a python vtk object instead

    # Not build in conda pcl
    ("MLSResult", "calculatePrincipleCurvatures"),
    ("MLSResult", "projectPointOrthogonalToPolynomialSurface"),
    ("MLSResult", "projectPointToMLSPlane"),
    ("MLSResult", "projectPointSimpleToPolynomialSurface"),
    ("MLSResult", "projectPoint"),
    ("MLSResult", "projectQueryPoint"),
    ("MLSResult", "getMLSCoordinates"),
    ("MLSResult", "getPolynomialValue"),
    ("MLSResult", "getPolynomialPartialDerivative"),
    ("MLSResult", "MLSResult"),
    ("MLSResult", "computeMLSSurface"),  # linking error with windows conda pcl
    # END Not build in conda pcl
    
    ("MovingLeastSquares", "getPolynomialFit"),  # deprecated in pcl
    ("MovingLeastSquares", "setPolynomialFit"),  # deprecated in pcl

    ("PCLHistogramVisualizer", "wasStopped"),  # only in vtk 5
    ("PCLHistogramVisualizer", "resetStoppedFlag"),  # only in vtk 5
    ("PCLVisualizerInteractorStyle", "vtkTypeMacro"),  # this is a macro?
    ("OctreePointCloudVoxelCentroid", "getVoxelCentroidAtPoint"),  # error: ‘genOctreeKeyforPoint’ was not declared in this scope

    ("RSDEstimation", "getHistograms"),  # must declare class for return value: boost::shared_ptr<std::vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf> > >
]
