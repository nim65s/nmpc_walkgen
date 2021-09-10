from libs cimport a

cdef public int b() except -1:
    print(a.myfunction(1,3))
    return 0

b()