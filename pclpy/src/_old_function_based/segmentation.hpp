#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/cpc_segmentation.h>

#include <pcl/features/don.h>

namespace py = pybind11;
using namespace pybind11::literals;


//template <typename T>
//py::object extractClustersConditional(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
//                                           bool (*func)(void),
//                                           float tolerance,
//                                           int min_cluster_size,
//                                           int max_cluster_size,
//                                           bool return_only_indices=false) {
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::ConditionalEuclideanClustering<T> ec;
//    ec.setConditionFunction (func);
//    ec.setClusterTolerance (tolerance);
//    ec.setMinClusterSize (min_cluster_size);
//    ec.setMaxClusterSize (max_cluster_size);
//    ec.setInputCloud (input_cloud);
//    ec.segment (cluster_indices);
//
//    return format_cluster_indices<T>(input_cloud, cluster_indices, return_only_indices);
//}

template <typename T>
pcl::CPCSegmentation<T> cpcSegmentation(std::map<uint32_t, boost::shared_ptr<pcl::Supervoxel<T>>> &supervoxel_clusters,
                           std::multimap<uint32_t, uint32_t> &supervoxel_adjacency,
                                     float voxel_resolution,
                                     float seed_resolution,
                                     float concavity_tolerance_threshold = 10,
                                     bool use_sanity_criterion = true,
                                     unsigned int max_cuts = 25,
                                     unsigned int cutting_min_segments = 400,
                                     float min_cut_score = 0.16,
                                     bool use_local_constrain = false,
                                     bool use_directed_cutting = true,
                                     bool use_clean_cutting = false,
                                     unsigned int ransac_iterations = 10000,
                                     float smoothness_threshold = 0.1,
                                     uint32_t min_segment_size = 0,
                                     bool use_extended_convexity = false
                                     ) {
    unsigned int k_factor = 0;
    if (use_extended_convexity)
        k_factor = 1;
    pcl::CPCSegmentation<T> cpc;
    cpc.setConcavityToleranceThreshold (concavity_tolerance_threshold);
    cpc.setSanityCheck (use_sanity_criterion);
    cpc.setCutting (max_cuts, cutting_min_segments, min_cut_score, use_local_constrain, use_directed_cutting, use_clean_cutting);
    cpc.setRANSACIterations (ransac_iterations);
    cpc.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
    cpc.setKFactor (k_factor);
    cpc.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
    cpc.setMinSegmentSize (min_segment_size);
    cpc.segment ();

    return cpc;
}

template <typename T, typename U>
py::object extractSupervoxelClusters(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                     boost::shared_ptr<pcl::PointCloud<U>> &normals,
                                     float voxel_resolution,
                                     float seed_resolution,
                                     float color_importance,
                                     float spatial_importance,
                                     float normal_importance,
                                     bool use_single_camera_transform,
                                     bool refine_supervoxels
                                     ) {
    pcl::SupervoxelClustering<T> sc (voxel_resolution, seed_resolution);
    sc.setColorImportance (color_importance);
    sc.setSpatialImportance (spatial_importance);
    sc.setNormalImportance (normal_importance);
    sc.setUseSingleCameraTransform (use_single_camera_transform);
    sc.setInputCloud (input_cloud);
    sc.setNormalCloud (normals);

    std::map<uint32_t, pcl::Supervoxel<T>::Ptr> supervoxel_clusters;
    sc.extract (supervoxel_clusters);

    if (refine_supervoxels) {
        sc.refineSupervoxels (2, supervoxel_clusters);
    }

    return py::make_tuple(py::cast(sc), py::cast(supervoxel_clusters));
}

template <typename T>
py::object extractClusters(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                           float tolerance,
                                           int min_cluster_size,
                                           int max_cluster_size,
                                           bool return_only_indices=false) {
    pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
    tree->setInputCloud (input_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<T> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    ec.extract (cluster_indices);
    return format_cluster_indices<T>(input_cloud, cluster_indices, return_only_indices);
}
template <typename T>
py::object APMF(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                           float slope=0.7,
                                           int max_window_size=33,
                                           float max_distance=10,
                                           float initial_distance=0.15,
                                           float cell_size=1,
                                           float base=2,
                                           bool exponential=true,
                                           int n_threads=0,
                                           bool return_only_indices=false) {
    pcl::ApproximateProgressiveMorphologicalFilter<T> apmf;
    apmf.setSlope (slope);
    apmf.setMaxWindowSize (max_window_size);
    apmf.setMaxDistance (max_distance);
    apmf.setInitialDistance (initial_distance);
    apmf.setCellSize (cell_size);
    apmf.setBase (base);
    apmf.setExponential (exponential);
    apmf.setNumberOfThreads (n_threads);
    apmf.setInputCloud (input_cloud);

    std::vector<int> ground;
    apmf.extract (ground);

    return vector_view(ground);
}


template <typename T, typename U>
py::object regionGrowing(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                           boost::shared_ptr<pcl::PointCloud<U>> &normals,
                                           int k_neighbours,
                                           int min_cluster_size,
                                           int max_cluster_size,
                                           float smoothness_threshold_degrees,
                                           float curvature_threshold = 1.0,
                                           bool return_only_indices=false) {
    pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
    tree->setInputCloud (input_cloud);

    pcl::RegionGrowing<T, U> reg;
    reg.setMinClusterSize (min_cluster_size);
    reg.setMaxClusterSize (max_cluster_size);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (k_neighbours);
    reg.setInputCloud (input_cloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smoothness_threshold_degrees / 180.0 * M_PI);
    reg.setCurvatureThreshold (curvature_threshold);

    std::vector<pcl::PointIndices> cluster_indices;
    reg.extract(cluster_indices);

    return format_cluster_indices<T>(input_cloud, cluster_indices, return_only_indices);
}

template <typename T, typename U>
py::object regionGrowingAt(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                           int index,
                                           boost::shared_ptr<pcl::PointCloud<U>> &normals,
                                           int k_neighbours,
                                           int min_cluster_size,
                                           int max_cluster_size,
                                           float smoothness_threshold_degrees,
                                           float curvature_threshold = 1.0) {
    pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
    tree->setInputCloud (input_cloud);

    pcl::RegionGrowing<T, U> reg;
    reg.setMinClusterSize (min_cluster_size);
    reg.setMaxClusterSize (max_cluster_size);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (k_neighbours);
    reg.setInputCloud (input_cloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smoothness_threshold_degrees / 180.0 * M_PI);
    reg.setCurvatureThreshold (curvature_threshold);

    pcl::PointIndices cluster_indices;
    reg.getSegmentFromPoint(index, cluster_indices);

    return vector_view(cluster_indices.indices);
}

py::array vector_view(std::vector<int> vector) {
    py::buffer_info buf = py::buffer_info(
        vector.data(),    /* Pointer to buffer */
        sizeof(int),                                    /* Size of one scalar */
        py::format_descriptor<int>::format(),           /* Python struct-style format descriptor */
        1,                                                /* Number of dimensions */
        {vector.size()},                                            /* Shape */
        {sizeof(int)}              /* Strides (in bytes) for each index */
    );
    return py::array(buf);
}

template<typename T>
py::object format_cluster_indices(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                                  std::vector<pcl::PointIndices> &cluster_indices,
                                  bool return_only_indices) {
    if (return_only_indices) {
        py::list out;
        for (pcl::PointIndices pi : cluster_indices) {
            out.attr("append")(vector_view(pi.indices));
        }
        return out;
    } else {
        std::vector<boost::shared_ptr<pcl::PointCloud<T>>> extracted_clusters;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<T>::Ptr cloud_cluster (new pcl::PointCloud<T>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (input_cloud->points[*pit]);
            cloud_cluster->width = (uint32_t) input_cloud->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            extracted_clusters.push_back(cloud_cluster);
        }
        return py::cast(extracted_clusters);
    }
}

template<typename T>
void defineSupervoxel(py::module &m, std::string const & suffix) {
    using ClassPtr = boost::shared_ptr<pcl::Supervoxel<T>>;
    py::class_<ClassPtr> cls(m, ("Supervoxel" + suffix).c_str());
}
template<typename T>
void defineCPCSegmentation(py::module &m, std::string const & suffix) {
    using Class = pcl::CPCSegmentation<T>;
    py::class_<Class> cls(m, ("CPCSegmentation" + suffix).c_str());
    cls.def("relabel_cloud", [](Class &cpc, pcl::PointCloud<pcl::PointXYZL>::Ptr &xyzl) {
                                return cpc.relabelCloud(*xyzl);});
//    cls.def("get_SV_adjacency_list ", &Class::getSVAdjacencyList);
}


template <typename T>
void defineSupervoxelClustering(py::module &m, std::string const & suffix) {
    using Class = pcl::SupervoxelClustering<T>;

    py::class_<Class> cls(m, ("SupervoxelClustering" + suffix).c_str());
    cls
      .def("extract", [](Class &__inst) {
        std::map<uint32_t, pcl::Supervoxel<T>::Ptr> supervoxel_clusters;
        __inst.extract(supervoxel_clusters);
        return supervoxel_clusters;
      })
      .def("refine_supervoxels", &Class::refineSupervoxels)
      .def("get_voxel_centroid_cloud", &Class::getVoxelCentroidCloud)
      .def("get_labeled_cloud", &Class::getLabeledCloud)
      .def("get_labeled_voxel_cloud", &Class::getLabeledVoxelCloud)
      .def("get_supervoxel_adjacency", [](Class &__inst) {
        std::multimap<uint32_t, uint32_t> label_adjacency;
        __inst.getSupervoxelAdjacency(label_adjacency);
        return py::cast(label_adjacency);
      })
      .def("make_supervoxel_normal_cloud", &Class::makeSupervoxelNormalCloud)
      .def("get_max_label", &Class::getMaxLabel);
}

template <typename T>
boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>>
differenceOfNormals(boost::shared_ptr<pcl::PointCloud<T>> &input_cloud,
                    boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &normals_large_scale,
                    boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &normals_small_scale
                    ) {
    pcl::DifferenceOfNormalsEstimation<T, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud (input_cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);

//    if (!don.initCompute ())
//    {
//        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
//    } else {
    don.computeFeature (*doncloud);
//    }
    return doncloud;
}

template <typename T>
void defineModuleFunctions(py::module &m) {
    m.def("cpc_segmentation", &cpcSegmentation<T>,
                                     "supervoxel_clusters"_a,
                                     "supervoxel_adjacency"_a,
                                     "voxel_resolution"_a,
                                     "seed_resolution"_a,
                                     "concavity_tolerance_threshold"_a=10,
                                     "use_sanity_criterion"_a=true,
                                     "max_cuts"_a=25,
                                     "cutting_min_segments"_a=400,
                                     "min_cut_score"_a=0.16,
                                     "use_local_constrain"_a=false,
                                     "use_directed_cutting"_a=true,
                                     "use_clean_cutting"_a=false,
                                     "ransac_iterations"_a=10000,
                                     "smoothness_threshold"_a=0.1,
                                     "min_segment_size"_a=0,
                                     "use_extended_convexity"_a=false);
}



void defineSegmentationClases(py::module &m) {
    defineSupervoxel<pcl::PointXYZ>(m, "XYZ");
    defineSupervoxel<pcl::PointXYZRGBA>(m, "XYZRGBA");
    defineSupervoxelClustering<pcl::PointXYZ>(m, "XYZ");
    defineSupervoxelClustering<pcl::PointXYZRGBA>(m, "XYZRGBA");
    defineCPCSegmentation<pcl::PointXYZ>(m, "XYZ");
    defineCPCSegmentation<pcl::PointXYZRGBA>(m, "XYZRGBA");

    defineModuleFunctions<pcl::PointXYZ>(m);
    defineModuleFunctions<pcl::PointXYZRGBA>(m);

    py::class_<std::multimap<uint32_t, uint32_t>> cls(m, "SupervoxelAdjacency");
}

