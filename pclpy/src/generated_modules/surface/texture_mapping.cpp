
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

#include <pcl/surface/texture_mapping.h>



template<typename PointInT>
void defineSurfaceTextureMapping(py::module &m, std::string const & suffix) {
    using Class = pcl::TextureMapping<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Octree = Class::Octree;
    using OctreePtr = Class::OctreePtr;
    using OctreeConstPtr = Class::OctreeConstPtr;
    using Camera = Class::Camera;
    using UvIndex = Class::UvIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("mapTexture2Mesh", &Class::mapTexture2Mesh, "tex_mesh"_a);
    cls.def("mapTexture2MeshUV", &Class::mapTexture2MeshUV, "tex_mesh"_a);
    cls.def("mapMultipleTexturesToMeshUV", &Class::mapMultipleTexturesToMeshUV, "tex_mesh"_a, "cams"_a);
    cls.def("isPointOccluded", &Class::isPointOccluded, "pt"_a, "octree"_a);
    cls.def("removeOccludedPoints", py::overload_cast<const PointCloudPtr &, PointCloudPtr &, const double, std::vector<int> &, std::vector<int> &> (&Class::removeOccludedPoints), "input_cloud"_a, "filtered_cloud"_a, "octree_voxel_size"_a, "visible_indices"_a, "occluded_indices"_a);
    cls.def("removeOccludedPoints", py::overload_cast<const pcl::TextureMesh &, pcl::TextureMesh &, const double> (&Class::removeOccludedPoints), "tex_mesh"_a, "cleaned_mesh"_a, "octree_voxel_size"_a);
    cls.def("removeOccludedPoints", py::overload_cast<const pcl::TextureMesh &, PointCloudPtr &, const double> (&Class::removeOccludedPoints), "tex_mesh"_a, "filtered_cloud"_a, "octree_voxel_size"_a);
    cls.def("sortFacesByCamera", &Class::sortFacesByCamera, "tex_mesh"_a, "sorted_mesh"_a, "cameras"_a, "octree_voxel_size"_a, "visible_pts"_a);
    cls.def("showOcclusions", py::overload_cast<const PointCloudPtr &, pcl::PointCloud<pcl::PointXYZI>::Ptr &, const double, const bool, const int> (&Class::showOcclusions), "input_cloud"_a, "colored_cloud"_a, "octree_voxel_size"_a, "show_nb_occlusions"_a=true, "max_occlusions"_a=4);
    cls.def("showOcclusions", py::overload_cast<pcl::TextureMesh &, pcl::PointCloud<pcl::PointXYZI>::Ptr &, double, bool, int> (&Class::showOcclusions), "tex_mesh"_a, "colored_cloud"_a, "octree_voxel_size"_a, "show_nb_occlusions"_a=true, "max_occlusions"_a=4);
    cls.def("textureMeshwithMultipleCameras", &Class::textureMeshwithMultipleCameras, "mesh"_a, "cameras"_a);
    cls.def("setF", &Class::setF, "f"_a);
    cls.def("setVectorField", &Class::setVectorField, "x"_a, "y"_a, "z"_a);
    cls.def("setTextureFiles", &Class::setTextureFiles, "tex_files"_a);
    cls.def("setTextureMaterials", &Class::setTextureMaterials, "tex_material"_a);
    cls.def("getPointUVCoordinates", py::overload_cast<const PointInT &, const Camera &, Eigen::Vector2f &> (&Class::getPointUVCoordinates), "pt"_a, "cam"_a, "UV_coordinates"_a);
        
}

void defineSurfaceTextureMappingFunctions(py::module &m) {
}

void defineSurfaceTextureMappingClasses(py::module &sub_module) {
    py::module sub_module_TextureMapping = sub_module.def_submodule("TextureMapping", "Submodule TextureMapping");
    defineSurfaceTextureMapping<pcl::PointXYZ>(sub_module_TextureMapping, "PointXYZ");
}