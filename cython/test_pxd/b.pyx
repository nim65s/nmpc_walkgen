from libs cimport a

cpdef public int b() except -1:
    print(a.myfunction(1,3))
    val = a.A()
    l = val.foo(1)
    print(l)
    return 0

b()
