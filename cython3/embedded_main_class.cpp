/* embedded_main.c */

/* This include file is automatically generated by Cython for 'public' functions. */
#include "nmpc_vel_ref_class_api.h"
#include <chrono>
#include <ctime>
#include <cstdio>
#include <string>

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
    if (PyImport_AppendInittab("nmpc_vel_ref_class", PyInit_nmpc_vel_ref_class) == -1) {
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

    float comx = -3.16e-3;
    float dcomx = 0.;
    float ddcomx = 0.;
    float comy = 1.237384291203724555e-03;
    float dcomy = 0.;
    float ddcomy = 0.;
    float comz = 8.786810585901939641e-01;
    float comq = 0.;
    float dcomq = 0.;
    float ddcomq = 0.;

    float footx = 1.86e-4;
    float footy = 0.085;
    float footq = 0.;
    std::string foot = "left";

    float vx = 0.;
    float vy = 0.;
    float vq = 0.; 
    std::string state = "D";   

    int nb_step = 10;

    import_nmpc_vel_ref_class();

    clock_t t;
    t = clock();

    Nmpc *nmpc = buildNmpc(comx, dcomx, ddcomx, comy, dcomy, ddcomy, comz, 
        footx, footy, footq, comq, dcomq, ddcomq, vx,  vy, vq,foot.c_str(),
        state.c_str());

    for (int i = 0; i < 8*nb_step; i++)
    {
        printf("iteration : %d\n",i);        
        if(i == 7)
        {
            set_velocity_referenceNmpc(nmpc,0.2,vy,vq);
            read_velocity_referenceNmpc(nmpc);
        }
        if(i == 8*nb_step-1) 
        {
            set_velocity_referenceNmpc(nmpc,vx,vy,vq);
            read_velocity_referenceNmpc(nmpc);
        }

        solveNmpc(nmpc,i);        
    }

    t = clock() - t;
    printf ("It took me %f seconds.\n",((float)t)/CLOCKS_PER_SEC);


    std::string path = "./nmpc_interpolated_cython_class.csv";
    interpolationNmpc(nmpc,path.c_str());

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
