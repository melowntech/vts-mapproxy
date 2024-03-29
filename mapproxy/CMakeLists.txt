# bootstrap build system
cmake_minimum_required(VERSION 3.3)
enable_language(Fortran)
project(vts-mapproxy)
# by default, this is OSS build (can be overrided by customer machinery)
set(BUILDSYS_DEFAULT_CUSTOMER_NAME oss)
set(BUILDSYS_CPP_STANDARD 17)
include(buildsys/cmake/buildsys.cmake)

enable_OpenMP()

find_package(Boost 1.46 REQUIRED
  COMPONENTS thread program_options filesystem system date_time
  serialization regex chrono iostreams)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

if(${Boost_VERSION_MAJOR}.${Boost_VERSION_MINOR} VERSION_GREATER_EQUAL 1.73)
  # Since Boost.Python and Boost.Iostreams in Boost >=1.73 still include
  # deprecated headers we need to silence compiler
  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS
    -DBOOST_ALLOW_DEPRECATED_HEADERS)
endif()

find_package(OpenCV REQUIRED)

find_package(GDAL REQUIRED)
include_directories(SYSTEM ${GDAL_INCLUDE_DIR})
find_package(PROJ REQUIRED)
include_directories(${PROJ_INCLUDE_DIR})
find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIR})

find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(CURL REQUIRED)
find_package(magic REQUIRED)
find_package(JsonCPP REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})

# needed by shtools
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(FFTW3 REQUIRED)

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
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

find_package(ICU COMPONENTS uc i18n REQUIRED)
include_directories(${ICU_INCLUDE_DIRS})

enable_python(3)
include_directories(${PYTHONLIBS_INCLUDE_DIRS})

find_package(tinyxml2 REQUIRED)
include_directories(${TINYXML2_INCLUDE_DIR})

find_package(Markdown REQUIRED)
include_directories(${Markdown_INCLUDE_DIR})

find_package(MGTS REQUIRED)
include_directories(${MGTS_INCLUDE_DIRS})
find_package(GLIB REQUIRED)
include_directories(${GLIB_INCLUDE_DIRS})

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
add_subdirectory(src/miniball)
add_subdirectory(src/qmf)
add_subdirectory(src/shtools)
add_subdirectory(src/semantic)

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
  add_custom_target(.vts ALL DEPENDS vts vts2vts)
endif()

# project
add_subdirectory(src/mapproxy)
add_subdirectory(src/tools)
add_subdirectory(src/tiling)
add_subdirectory(src/rf-mask)
add_subdirectory(src/generatevrtwo)
add_subdirectory(src/calipers)
add_subdirectory(src/setup-resource)

# for testing
add_subdirectory(src/utility/tools EXCLUDE_FROM_ALL)

set(semantic-tools-INSTALL-COMPONENT tools)
add_subdirectory(src/semantic/tools)
