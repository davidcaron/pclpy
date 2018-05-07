
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

#pragma warning(disable : 4996)
#include <pcl/sample_consensus/model_types.h>



void defineSampleConsensusModelTypesFunctions(py::module &m) {
}

void defineSampleConsensusModelTypesClasses(py::module &sub_module) {
    defineSampleConsensusModelTypesFunctions(sub_module);
    py::enum_<pcl::SacModel>(sub_module, "SacModel")
        .value("SACMODEL_PLANE", pcl::SacModel::SACMODEL_PLANE)
        .value("SACMODEL_LINE", pcl::SacModel::SACMODEL_LINE)
        .value("SACMODEL_CIRCLE2D", pcl::SacModel::SACMODEL_CIRCLE2D)
        .value("SACMODEL_CIRCLE3D", pcl::SacModel::SACMODEL_CIRCLE3D)
        .value("SACMODEL_SPHERE", pcl::SacModel::SACMODEL_SPHERE)
        .value("SACMODEL_CYLINDER", pcl::SacModel::SACMODEL_CYLINDER)
        .value("SACMODEL_CONE", pcl::SacModel::SACMODEL_CONE)
        .value("SACMODEL_TORUS", pcl::SacModel::SACMODEL_TORUS)
        .value("SACMODEL_PARALLEL_LINE", pcl::SacModel::SACMODEL_PARALLEL_LINE)
        .value("SACMODEL_PERPENDICULAR_PLANE", pcl::SacModel::SACMODEL_PERPENDICULAR_PLANE)
        .value("SACMODEL_PARALLEL_LINES", pcl::SacModel::SACMODEL_PARALLEL_LINES)
        .value("SACMODEL_NORMAL_PLANE", pcl::SacModel::SACMODEL_NORMAL_PLANE)
        .value("SACMODEL_NORMAL_SPHERE", pcl::SacModel::SACMODEL_NORMAL_SPHERE)
        .value("SACMODEL_REGISTRATION", pcl::SacModel::SACMODEL_REGISTRATION)
        .value("SACMODEL_REGISTRATION_2D", pcl::SacModel::SACMODEL_REGISTRATION_2D)
        .value("SACMODEL_PARALLEL_PLANE", pcl::SacModel::SACMODEL_PARALLEL_PLANE)
        .value("SACMODEL_NORMAL_PARALLEL_PLANE", pcl::SacModel::SACMODEL_NORMAL_PARALLEL_PLANE)
        .value("SACMODEL_STICK", pcl::SacModel::SACMODEL_STICK)
        .export_values();
}