
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

class A
{
    public:
      int add() { return 0; }
      template<typename T>
      T add() { return T(); }
};

PYBIND11_MODULE(pcl, m) {
    py::class_<A> cls(m, "A");
    cls.def("add", ([](A &a){return (a.add());}));
}
