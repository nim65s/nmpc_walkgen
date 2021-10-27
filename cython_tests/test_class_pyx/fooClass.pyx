# distutils: language=c++

from math import sin

cdef public class Foo[object Foo, type fooType]:
    cdef double a,b  

    def __cinit__(self, double a, double b):
        self.a = a
        self.b = b  

    cdef double bar(self, double c):
        self.a = sin(self.a*c)  

cdef api Foo buildFoo(double a, double b):
    return Foo(a,b)

cdef api void foobar(Foo foo, double d):
    foo.bar(d)