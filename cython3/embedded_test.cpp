/* embedded_main.c */

/* This include file is automatically generated by Cython for 'public' functions. */
#include "test.h"

#ifdef __cplusplus
extern "C" {
#endif

int
main(int argc, char *argv[])
{
    PyObject *pmodule;
    wchar_t *program;

    program = Py_DecodeLocale(argv[0], NULL);
    if (program == NULL) {
        fprintf(stderr, "Fatal error: cannot decode argv[0], got %d arguments\n", argc);
        exit(1);
    }

    /* Add a built-in module, before Py_Initialize */
    if (PyImport_AppendInittab("test", PyInit_test) == -1) {
        fprintf(stderr, "Error: could not extend in-built modules table\n");
        exit(1);
    }

    /* Pass argv[0] to the Python interpreter */
    Py_SetProgramName(program);

    /* Initialize the Python interpreter.  Required.
       If this step fails, it will be a fatal error. */
    Py_Initialize();

    /* Optionally import the module; alternatively,
       import can be deferred until the b script
       imports it. */
    PyRun_SimpleString(
       "import sys\n"
       "sys.path.append('')\n"
    );
    pmodule = PyImport_ImportModule("test");
    if (!pmodule) {
        PyErr_Print();
        fprintf(stderr, "Error: could not import module 'test'\n");
        goto exit_with_error;
    }

/*    test(100,20);*/

    /* Now call into your module code. */
    if (test(9,1) < 0) {
        PyErr_Print();
        fprintf(stderr, "Error in Python code, exception was printed.\n");
        goto exit_with_error;
    }

    /* ... */

    /* Clean up after using CPython. */
    PyMem_RawFree(program);
    Py_Finalize();

    return 0;

    /* Clean up in the error cases above. */
exit_with_error:
    PyMem_RawFree(program);
    Py_Finalize();
    return 1;
}

#ifdef __cplusplus
}
#endif
