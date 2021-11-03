cmake_minimum_required(VERSION 3.16)

set(PROJECT_NAME "cythonmpcwg")
set(PROJECT_DESCRIPTION "NMPC walkgen in Cython")
set(PROJECT_URL "https://github.com/imaroger/nmpc_walkgen")
#set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/base.cmake)
include(cmake/python.cmake)

project(${PROJECT_NAME} CXX)

FINDPYTHON()
FIND_NUMPY()

add_custom_command(OUTPUT
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/combinedqp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/combinedqp${PYTHON_SOABI}
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/base${PYTHON_SOABI}
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/helper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/helper${PYTHON_SOABI}
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/interpolation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/interpolation${PYTHON_SOABI}
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/__init__.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wpg/__init__${PYTHON_SOABI}
    ${CMAKE_CURRENT_SOURCE_DIR}/nmpc_vel_ref_class_api.h
    ${CMAKE_CURRENT_SOURCE_DIR}/nmpc_vel_ref_class.h
    ${CMAKE_CURRENT_SOURCE_DIR}/nmpc_vel_ref_class.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/nmpc_vel_ref_class${PYTHON_SOABI}
    COMMAND ${PYTHON_EXECUTABLE} setup_class.py build_ext --inplace
    DEPENDS
    wpg/combinedqp.pxd
    wpg/combinedqp.py
    wpg/base.pxd
    wpg/base.py
    wpg/helper.pxd
    wpg/helper.py
    wpg/interpolation.pxd
    wpg/interpolation.py
    wpg/__init__.py
    nmpc_vel_ref_class.pyx
    setup_class.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

set(${PROJECT_NAME}_SOURCES
    embedded_main_class.cpp
    nmpc_vel_ref_class.cpp
    )

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME}
        EXPORT ${TARGETS_EXPORT_NAME}
        DESTINATION bin)
