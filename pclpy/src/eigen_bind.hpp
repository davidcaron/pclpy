#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

namespace py = pybind11;
using namespace pybind11::literals;

void defineQuaternion(py::module &m)
{
    using Class = Eigen::Quaternionf;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Quaternionf");
    cls.def(py::init<>());
    // todo: make this usable...
    // For now, this enables compilation because Quaternion is not in pybind11/eigen.h
}

//void defineVectorXf(py::module & m) {
//    /* Bind MatrixXd (or some other Eigen type) to Python */
//    typedef Eigen::MatrixXf Matrix;
//
//    typedef Matrix::Scalar Scalar;
////    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;
//    bool rowMajor = true;
//
//    py::class_<Matrix> cls(m, "Matrix", py::buffer_protocol());
//    cls.def("__init__", [](Matrix &m, py::buffer b) {
//        typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;
//
//        /* Request a buffer descriptor from Python */
//        py::buffer_info info = b.request();
//
//        /* Some sanity checks ... */
//        if (info.format != py::format_descriptor<Scalar>::format())
//            throw std::runtime_error("Incompatible format: expected a float array!");
//
//        if (info.ndim != 2)
//            throw std::runtime_error("Incompatible buffer dimension!");
//
//        auto strides = Strides(
//            info.strides[0] / (py::ssize_t)sizeof(Scalar),
////            info.strides[rowMajor ? 0 : 1] / sizeof(Scalar),
//            info.strides[1] / (py::ssize_t)sizeof(Scalar));
////            info.strides[rowMajor ? 1 : 0] / sizeof(Scalar));
//
//        auto map = Eigen::Map<Matrix, 0, Strides>(
//            static_cast<Scalar *>(info.ptr), info.shape[0], info.shape[1], strides);
//
////        new (&m) Eigen::Map<Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic>> (static_cast<Scalar *>(info.ptr),
////            info.shape[0],info.shape[1]);
//
//        new (&m) Matrix(map);
//    });
//    cls.def_buffer([](Matrix &m) -> py::buffer_info {
//        return py::buffer_info(
//            m.data(),                /* Pointer to buffer */
//            sizeof(Scalar),          /* Size of one scalar */
//            /* Python struct-style format descriptor */
//            py::format_descriptor<Scalar>::format(),
//            /* Number of dimensions */
//            2,
//            /* Buffer dimensions */
//            { m.rows(),
//              m.cols() },
//            /* Strides (in bytes) for each index */
//            { sizeof(Scalar) * (m.cols()),
//              sizeof(Scalar) * (1) }
//        );
//     });
//
//}

void defineVectorXf(py::module &m)
{
    /* Bind MatrixXd (or some other Eigen type) to Python */
    typedef Eigen::VectorXf VectorXf;

    typedef VectorXf::Scalar Scalar;

    py::class_<VectorXf, boost::shared_ptr<VectorXf>> cls(m, "VectorXf", py::buffer_protocol());
    cls.def(py::init<>());
    cls.def("__init__", [](VectorXf &v, py::buffer b) {
        typedef Eigen::Stride<Eigen::Dynamic, 1> Strides;

        /* Request a buffer descriptor from Python */
        py::buffer_info info = b.request();

        /* Some sanity checks ... */
        if (info.format != py::format_descriptor<Scalar>::format())
            throw std::runtime_error("Incompatible format: expected a float array!");

        if (info.ndim != 1)
            throw std::runtime_error("Incompatible buffer dimension!");

        auto strides = Strides((py::ssize_t)sizeof(Scalar), 1);

        auto map = Eigen::Map<VectorXf, 0, Strides>(
            static_cast<Scalar *>(info.ptr), info.shape[0], strides);

        //        new (&m) Eigen::Map<Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic>> (static_cast<Scalar *>(info.ptr),
        //            info.shape[0],info.shape[1]);

        new (&v) VectorXf(map);
    });
    cls.def_buffer([](VectorXf &v) -> py::buffer_info {
        return py::buffer_info(
            v.data(),       /* Pointer to buffer */
            sizeof(Scalar), /* Size of one scalar */
            /* Python struct-style format descriptor */
            py::format_descriptor<Scalar>::format(),
            /* Number of dimensions */
            1,
            /* Buffer dimensions */
            {v.size()},
            /* Strides (in bytes) for each index */
            {sizeof(Scalar)});
    });
}

void defineEigenClasses(py::module &m)
{
    defineQuaternion(m);
    defineVectorXf(m);
}