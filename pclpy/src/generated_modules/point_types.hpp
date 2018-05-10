
#include <pcl/point_types.h>



void definePointTypesFunctions(py::module &m) {
}

void definePointTypesClasses(py::module &sub_module) {
    py::enum_<pcl::BorderTrait>(sub_module, "BorderTrait")
        .value("BORDER_TRAIT__OBSTACLE_BORDER", pcl::BorderTrait::BORDER_TRAIT__OBSTACLE_BORDER)
        .value("BORDER_TRAIT__SHADOW_BORDER", pcl::BorderTrait::BORDER_TRAIT__SHADOW_BORDER)
        .value("BORDER_TRAIT__VEIL_POINT", pcl::BorderTrait::BORDER_TRAIT__VEIL_POINT)
        .value("BORDER_TRAIT__SHADOW_BORDER_TOP", pcl::BorderTrait::BORDER_TRAIT__SHADOW_BORDER_TOP)
        .value("BORDER_TRAIT__SHADOW_BORDER_RIGHT", pcl::BorderTrait::BORDER_TRAIT__SHADOW_BORDER_RIGHT)
        .value("BORDER_TRAIT__SHADOW_BORDER_BOTTOM", pcl::BorderTrait::BORDER_TRAIT__SHADOW_BORDER_BOTTOM)
        .value("BORDER_TRAIT__SHADOW_BORDER_LEFT", pcl::BorderTrait::BORDER_TRAIT__SHADOW_BORDER_LEFT)
        .value("BORDER_TRAIT__OBSTACLE_BORDER_TOP", pcl::BorderTrait::BORDER_TRAIT__OBSTACLE_BORDER_TOP)
        .value("BORDER_TRAIT__OBSTACLE_BORDER_RIGHT", pcl::BorderTrait::BORDER_TRAIT__OBSTACLE_BORDER_RIGHT)
        .value("BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM", pcl::BorderTrait::BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM)
        .value("BORDER_TRAIT__OBSTACLE_BORDER_LEFT", pcl::BorderTrait::BORDER_TRAIT__OBSTACLE_BORDER_LEFT)
        .value("BORDER_TRAIT__VEIL_POINT_TOP", pcl::BorderTrait::BORDER_TRAIT__VEIL_POINT_TOP)
        .value("BORDER_TRAIT__VEIL_POINT_RIGHT", pcl::BorderTrait::BORDER_TRAIT__VEIL_POINT_RIGHT)
        .value("BORDER_TRAIT__VEIL_POINT_BOTTOM", pcl::BorderTrait::BORDER_TRAIT__VEIL_POINT_BOTTOM)
        .value("BORDER_TRAIT__VEIL_POINT_LEFT", pcl::BorderTrait::BORDER_TRAIT__VEIL_POINT_LEFT)
        .export_values();
}