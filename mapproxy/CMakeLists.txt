# bootstrap build system
cmake_minimum_required(VERSION 2.8.11)
include(buildsys/cmake/buildsys.cmake)

# project
setup_project(mapproxy "${BUILDSYS_PACKAGE_VERSION}")

find_package(Boost 1.46 REQUIRED
  COMPONENTS thread program_options filesystem system date_time
             serialization regex chrono iostreams)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

find_package(OpenCV 2.4 REQUIRED)

find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIR})
find_package(Proj REQUIRED)
include_directories(${PROJ_INCLUDE_DIR})
find_package(GeographicLib REQUIRED)
include_directories(${GEOGRAPHICLIB_INCLUDE_DIR})

find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(CURL REQUIRED)

find_package(MySQL++ REQUIRED)
add_definitions(${MYSQL++_DEFINITIONS})
include_directories(${MYSQL++_INCLUDE_DIR})

find_package(MHD REQUIRED)
add_definitions(${MHD_DEFINITIONS})
include_directories(${MHD_INCLUDE_DIR})

find_package(OpenMesh 2.3.1 REQUIRED)
include_directories(${OPENMESH_INCLUDE_DIR})
link_directories(${OPENMESH_LIBRARY_DIR})

# dependencies
add_subdirectory(src/dbglog)
add_subdirectory(src/utility)
add_subdirectory(src/math)
add_subdirectory(src/geometry)
add_subdirectory(src/imgproc)
add_subdirectory(src/geo)
add_subdirectory(src/service)
add_subdirectory(src/optics)
add_subdirectory(src/cvision)
add_subdirectory(src/jsoncpp)
add_subdirectory(src/half)
add_subdirectory(src/gdal-drivers)

# vts libraries
add_subdirectory(src/vts-libs)
# force build of vts tool
add_custom_target(.vts ALL DEPENDS vts)

# project
add_subdirectory(src/mapproxy)
add_subdirectory(src/dem-tiling)
