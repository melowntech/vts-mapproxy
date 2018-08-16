# bootstrap build system
cmake_minimum_required(VERSION 2.8.11)
project(vts-mapproxy)
# by default, this is OSS build (can be overrided by customer machinery)
set(BUILDSYS_DEFAULT_CUSTOMER_NAME oss)
include(buildsys/cmake/buildsys.cmake)

enable_OpenMP()

find_package(Boost 1.46 REQUIRED
  COMPONENTS thread program_options filesystem system date_time
             serialization regex chrono iostreams)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(GDAL REQUIRED)
include_directories(SYSTEM ${GDAL_INCLUDE_DIR})
find_package(Proj REQUIRED)
include_directories(${PROJ_INCLUDE_DIR})
find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIR})

find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(CURL REQUIRED)
find_package(magic REQUIRED)
find_package(JsonCPP REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})

# Protobuf: needed by mvt ogr driver
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

# Sqlite3: needed by mvt ogr driver (mbtiles support)
find_package(Sqlite3 REQUIRED)
link_directories(${Sqlite3_LIBRARY_DIRS})
include_directories(${Sqlite3_INCLUDE_DIRS})

find_package(OpenMesh 2.3.1 REQUIRED)
include_directories(${OPENMESH_INCLUDE_DIR})
link_directories(${OPENMESH_LIBRARY_DIR})

find_package(LibProc REQUIRED)
include_directories(${LIBPROC_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(ICU COMPONENTS uc i18n REQUIRED)
include_directories(${ICU_INCLUDE_DIRS})

enable_python(2.7)
include_directories(${PYTHONLIBS_INCLUDE_DIRS})

# dependencies
add_subdirectory(src/dbglog)
add_subdirectory(src/utility)
add_subdirectory(src/jsoncpp)
add_subdirectory(src/math)
add_subdirectory(src/geometry)
add_subdirectory(src/imgproc)
add_subdirectory(src/service)
add_subdirectory(src/geo)
add_subdirectory(src/half)
add_subdirectory(src/gdal-drivers)
add_subdirectory(src/http)

# python stuff
set(PYDBGLOG_STATIC TRUE)
add_subdirectory(src/pydbglog)
add_subdirectory(src/pysupport)

# vts libraries
set(VTS_REGISTRY_PYTHON true)

# vts libraries
add_subdirectory(src/vts-libs)

if(NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-tools)
  # force build of vts tool
  add_custom_target(.vts ALL DEPENDS vts vts02vts vts2vts vts0)
endif()

# project
add_subdirectory(src/mapproxy)
add_subdirectory(src/tools)
add_subdirectory(src/tiling)
add_subdirectory(src/rf-mask)
add_subdirectory(src/generatevrtwo)
add_subdirectory(src/calipers)
add_subdirectory(src/setup-resource)
