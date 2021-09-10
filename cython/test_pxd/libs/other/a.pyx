cpdef int myfunction(int x, int y=2):
    a = x - y
    return a + x * y

cdef double _helper(double a):
    return a + 1