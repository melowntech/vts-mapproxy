# test program
define_module(BINARY mapproxy
  DEPENDS
  pysupport pydbglog
  vts-libs>=2.0 http>=1.0 service>=1.6 gdal-drivers>=1.13 geometry>=1.7

  Boost_SERIALIZATION
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  Boost_PYTHON
  )

set(mapproxy_SOURCES
  error.hpp
  resource.hpp resource.cpp pyresource.cpp
  support/metatile.hpp support/metatile.cpp
  support/mesh.hpp support/mesh.cpp
  support/geo.hpp support/geo.cpp
  support/coverage.hpp support/coverage.cpp
  support/tileindex.hpp support/tileindex.cpp
  support/fileclass.hpp support/fileclass.cpp

  resourcebackend.hpp
  resourcebackend/resourcebackend.cpp
  resourcebackend/factory.hpp
  resourcebackend/conffile.hpp resourcebackend/conffile.cpp
  resourcebackend/python.hpp resourcebackend/python.cpp

  generator.hpp
  generator/metatile.hpp generator/metatile.cpp
  generator/demregistry.hpp generator/demregistry.cpp
  generator/generator.cpp
  generator/factory.hpp
  generator/tms-raster.hpp generator/tms-raster.cpp
  generator/tms-raster-remote.hpp generator/tms-raster-remote.cpp
  generator/tms-bing.hpp generator/tms-bing.cpp
  generator/tms-windyty.hpp generator/tms-windyty.cpp
  generator/tms-raster-patchwork.hpp generator/tms-raster-patchwork.cpp

  generator/surface.hpp generator/surface.cpp
  generator/surface-spheroid.hpp generator/surface-spheroid.cpp
  generator/surface-dem.hpp generator/surface-dem.cpp

  generator/geodatavectorbase.hpp generator/geodatavectorbase.cpp
  generator/geodata-vector-tiled.hpp generator/geodata-vector-tiled.cpp
  generator/geodata-vector.hpp generator/geodata-vector.cpp

  sink.hpp sink.cpp

  fileinfo.hpp fileinfo.cpp
  core.hpp core.cpp

  gdalsupport.hpp gdalsupport/gdalsupport.cpp
  gdalsupport/types.hpp
  gdalsupport/requests.hpp gdalsupport/requests.cpp
  gdalsupport/process.hpp gdalsupport/process.cpp
  gdalsupport/datasetcache.hpp gdalsupport/datasetcache.cpp
  gdalsupport/operations.hpp gdalsupport/operations.cpp

  main.cpp
  )

set(mapproxy_BROWSER_SOURCES
  browser2d.hpp browser2d.cpp
  )
file_to_cpp(mapproxy_BROWSER_SOURCES
  browser2d::index_html
  browser2d/index.html)
file_to_cpp(mapproxy_BROWSER_SOURCES
  browser2d::index_js
  browser2d/index.js)


add_executable(mapproxy ${mapproxy_SOURCES} ${mapproxy_BROWSER_SOURCES})
target_link_libraries(mapproxy ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy)
set_target_version(mapproxy ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy RUNTIME DESTINATION bin
  COMPONENT main)
