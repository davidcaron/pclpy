
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/texture_mapping.h>



template<typename PointInT>
void defineSurfaceTextureMapping(py::module &m, std::string const & suffix) {
    using Class = TextureMapping<PointInT>;
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
    cls.def("set_f", &Class::setF);
    cls.def("set_vector_field", &Class::setVectorField);
    cls.def("set_texture_files", &Class::setTextureFiles);
    cls.def("set_texture_materials", &Class::setTextureMaterials);
    cls.def("remove_occluded_points", py::overload_cast<const PointCloudPtr &, PointCloudPtr &, const double, std::vector<int> &, std::vector<int> &> (&Class::removeOccludedPoints));
    cls.def("remove_occluded_points", py::overload_cast<const pcl::TextureMesh &, pcl::TextureMesh &, const double> (&Class::removeOccludedPoints));
    cls.def("remove_occluded_points", py::overload_cast<const pcl::TextureMesh &, PointCloudPtr &, const double> (&Class::removeOccludedPoints));
    cls.def("show_occlusions", py::overload_cast<const PointCloudPtr &, pcl::PointCloud<pcl::PointXYZI>::Ptr &, const double, const bool, const int> (&Class::showOcclusions));
    cls.def("show_occlusions", py::overload_cast<pcl::TextureMesh &, pcl::PointCloud<pcl::PointXYZI>::Ptr &, double, bool, int> (&Class::showOcclusions));
    cls.def("map_texture2_mesh", &Class::mapTexture2Mesh);
    cls.def("map_texture2_mesh_uv", &Class::mapTexture2MeshUV);
    cls.def("map_multiple_textures_to_mesh_uv", &Class::mapMultipleTexturesToMeshUV);
    cls.def("is_point_occluded", &Class::isPointOccluded);
    cls.def("sort_faces_by_camera", &Class::sortFacesByCamera);
    cls.def("texture_meshwith_multiple_cameras", &Class::textureMeshwithMultipleCameras);
        
}

void defineSurfaceTextureMappingClasses(py::module &sub_module) {
    py::module sub_module_TextureMapping = sub_module.def_submodule("TextureMapping", "Submodule TextureMapping");
    defineSurfaceTextureMapping<PointXYZ>(sub_module_TextureMapping, "PointXYZ");
}