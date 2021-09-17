#!/bin/bash

make clean
make nmpc_vel_ref.cpp
sed -i '2i#include "Python.h"' nmpc_vel_ref.h
make test
./test