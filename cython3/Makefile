# Makefile for creating our standalone Cython program
PYTHON := python3
PYVERSION := $(shell $(PYTHON) -c "import sys; print(sys.version[:3])")
PYPREFIX := $(shell $(PYTHON) -c "import sys; print(sys.prefix)")

INCDIR := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_python_inc())")
PLATINCDIR := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_python_inc(plat_specific=True))")
LIBDIR1 := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
LIBDIR2 := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_config_var('LIBPL'))")
PYLIB := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_config_var('LIBRARY')[3:-2])")

CC := $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('CC'))")
LINKCC := $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('LINKCC'))")
LINKFORSHARED := $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('LINKFORSHARED'))")
LIBS := $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('LIBS'))")
SYSLIBS :=  $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('SYSLIBS'))")


.PHONY: paths all clean test

paths:
	@echo "PYTHON=$(PYTHON)"
	@echo "PYVERSION=$(PYVERSION)"
	@echo "PYPREFIX=$(PYPREFIX)"
	@echo "INCDIR=$(INCDIR)"
	@echo "PLATINCDIR=$(PLATINCDIR)"
	@echo "LIBDIR1=$(LIBDIR1)"
	@echo "LIBDIR2=$(LIBDIR2)"
	@echo "PYLIB=$(PYLIB)"
	@echo "CC=$(CC)"
	@echo "LINKCC=$(LINKCC)"
	@echo "LINKFORSHARED=$(LINKFORSHARED)"
	@echo "LIBS=$(LIBS)"
	@echo "SYSLIBS=$(SYSLIBS)"

nmpc_vel_ref: nmpc_vel_ref.o embedded_main.o
	$(LINKCC) -O3 -o $@ $^ -L$(LIBDIR1) -L$(LIBDIR2) -l$(PYLIB) $(LIBS) $(SYSLIBS) $(LINKFORSHARED)

test: test.o embedded_test.o
	$(LINKCC) -o $@ $^ -L$(LIBDIR1) -L$(LIBDIR2) -l$(PYLIB) $(LIBS) $(SYSLIBS) $(LINKFORSHARED)

nmpc_vel_ref_online: nmpc_vel_ref_online.o embedded_main_online.o
	$(LINKCC) -O3 -o $@ $^ -L$(LIBDIR1) -L$(LIBDIR2) -l$(PYLIB) $(LIBS) $(SYSLIBS) $(LINKFORSHARED)

nmpc_vel_ref_class: nmpc_vel_ref_class.o embedded_main_class.o
	$(LINKCC) -O3 -o $@ $^ -L$(LIBDIR1) -L$(LIBDIR2) -l$(PYLIB) $(LIBS) $(SYSLIBS) $(LINKFORSHARED)


%.o: %.cpp
	$(CC) -O3 -c $^ -I$(INCDIR) -I$(PLATINCDIR)


nmpc_vel_ref.cpp: nmpc_vel_ref.pyx wpg/combinedqp.py wpg/base.py wpg/helper.py wpg/interpolation.py
	$(PYTHON) setup.py build_ext --inplace

test.cpp: test.pyx
	$(PYTHON) setup_test.py build_ext --inplace

nmpc_vel_ref_online.cpp: nmpc_vel_ref_online.pyx wpg/combinedqp.py wpg/base.py wpg/helper.py wpg/interpolation.py
	$(PYTHON) setup_online.py build_ext --inplace

nmpc_vel_ref_class.cpp: nmpc_vel_ref_class.pyx wpg/combinedqp.py wpg/base.py wpg/helper.py wpg/interpolation.py
	$(PYTHON) setup_class.py build_ext --inplace	


clean:
	@echo Cleaning ...
	@rm -rf *~ *.o *.so core core.* test nmpc_vel_ref nmpc_vel_ref.cpp nmpc_vel_ref_class nmpc_vel_ref_class.cpp nmpc_vel_ref_online nmpc_vel_ref_online.cpp test.cpp *.h libs/*.cpp build libs/*.so libs/__pycache__ wpg/*.cpp build wpg/*.so wpg/__pycache__ 
