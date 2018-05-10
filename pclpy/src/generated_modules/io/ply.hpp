
#include <pcl/io/ply/ply.h>



void defineIoPlyFunctions(py::module &m) {
}

void defineIoPlyClasses(py::module &sub_module) {
    py::enum_<pcl::io::ply::format>(sub_module, "format")
        .value("ascii_format", pcl::io::ply::format::ascii_format)
        .value("binary_little_endian_format", pcl::io::ply::format::binary_little_endian_format)
        .value("binary_big_endian_format", pcl::io::ply::format::binary_big_endian_format)
        .value("unknown", pcl::io::ply::format::unknown)
        .export_values();
}