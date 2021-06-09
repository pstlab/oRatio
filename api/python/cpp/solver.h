#define PY_SSIZE_T_CLEAN
#include <Python.h>

#ifdef __cplusplus
extern "C"
{
#endif
    static PyObject *new_instance(PyObject *self, PyObject *args);
    static PyObject *dispose(PyObject *self, PyObject *args);

    static PyObject *read_riddle(PyObject *self, PyObject *args);

    static PyObject *solve(PyObject *self, PyObject *args);
#ifdef __cplusplus
}
#endif

static PyMethodDef methods[] = {
    {"new_instance", new_instance, METH_VARARGS, "Creates a new oRatio solver instance."},
    {"dispose", dispose, METH_VARARGS, "Creates a new oRatio solver instance."},
    {"read", read_riddle, METH_VARARGS, "Creates a new oRatio solver instance."},
    {"solve", solve, METH_VARARGS, "Creates a new oRatio solver instance."},
    {NULL, NULL, 0, NULL} /* Sentinel */
};

static struct PyModuleDef module = {PyModuleDef_HEAD_INIT, "oratio", NULL, -1, methods};

PyMODINIT_FUNC PyInit_oratio(void) { return PyModule_Create(&module); }