#!/bin/bash

make clean
make $1.cpp
# sed -i '2i#include "Python.h"' $1.h
sed -i '2i#include "numpy/arrayobject.h"' $1.h
make $1
./$1