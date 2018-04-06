
template <typename PointT>
void defineSuperVoxelClustering(py::module &m) {

    py::class_<boost::shared_ptr<pcl::SupervoxelClustering<T>>> supervoxelclustering(m, "SupervoxelClustering");
    supervoxelclustering
      .def("extract", [](pcl::SupervoxelClustering<T> &__inst) {
        std::map<uint32_t, pcl::Supervoxel<T>::Ptr> supervoxel_clusters;
        __inst.extract(supervoxel_clusters);
        return supervoxel_clusters;
      })
      .def("refineSupervoxels", [](pcl::SupervoxelClustering<T> &__inst, int num_itr, Supervoxel<PointT>::Ptr> supervoxel_clusters) {
        std::map<uint32_t, typename Supervoxel<PointT>::Ptr> supervoxel_clusters;
        __inst.refineSupervoxels(num_itr, supervoxel_clusters);
        return supervoxel_clusters;
      })
      .def("getVoxelCentroidCloud", &SupervoxelClustering::getVoxelCentroidCloud)
      .def("getLabeledCloud", &SupervoxelClustering::getLabeledCloud)
      .def("getLabeledVoxelCloud", &SupervoxelClustering::getLabeledVoxelCloud)
//      .def("getSupervoxelAdjacencyList", [](SupervoxelClustering &__inst) {
//        pcl::SupervoxelClustering::VoxelAdjacencyList adjacency_list_arg;
//        __inst.getSupervoxelAdjacencyList(adjacency_list_arg);
//        return adjacency_list_arg;
//      })
      .def("getSupervoxelAdjacency", [](pcl::SupervoxelClustering<T> &__inst) {
        std::multimap<uint32_t, uint32_t> label_adjacency;
        __inst.getSupervoxelAdjacency(label_adjacency);
        return label_adjacency;
      })
      .def("makeSupervoxelNormalCloud", [](SupervoxelClustering &__inst) {
        std::map<uint32_t, typename Supervoxel<PointT>::Ptr> supervoxel_clusters;
        auto __ret = __inst.makeSupervoxelNormalCloud(supervoxel_clusters);
        return std::make_tuple(__ret, supervoxel_clusters);
      })
      .def("getMaxLabel", &SupervoxelClustering::getMaxLabel);

}