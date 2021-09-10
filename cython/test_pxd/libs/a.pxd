# distutils: language=c++

# import cython

# cpdef inline int myfunction(int x, int y):
#     a = x - y
#     return a + x * y

# cdef inline double _helper(double a):
#     return a + 1

# cdef class Myclass:
#     cdef public int a
#     def __init__(self, b=0):
#          self.a = 3
#          self.b = b

    # cpdef int foo(self, double x) except *:
    #     # print(x + _helper(1.0))
    #     return(0)

cpdef int myfunction(int x, int y=*)
cdef double _helper(double a)

cdef class A:
    cdef public int a, b
    cpdef foo(self, double x)