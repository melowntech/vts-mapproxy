# ------------------------------------------------------------------------
# core mapproxy library
set(mapproxy-core_SOURCES
  error.hpp
  resource.hpp resource.cpp pyresource.cpp
  support/metatile.hpp support/metatile.cpp
  support/mesh.hpp support/mesh.cpp
  support/geo.hpp support/geo.cpp
  support/coverage.hpp support/coverage.cpp
  support/tileindex.hpp support/tileindex.cpp
  support/fileclass.hpp support/fileclass.cpp
  support/serialization.hpp support/serialization.cpp
  support/aborter.hpp
  support/glob.hpp support/glob.cpp
  support/tilejson.hpp support/tilejson.cpp
  support/cesiumconf.hpp support/cesiumconf.cpp
  support/wmts.hpp support/wmts.cpp

  support/mmapped/tileindex.hpp support/mmapped/tileindex.cpp
  support/mmapped/qtree.hpp support/mmapped/qtree.cpp
  support/mmapped/memory.hpp support/mmapped/memory-impl.hpp
  support/mmapped/tileflags.hpp
  support/mmapped/qtree-rasterize.hpp

  support/mmapped/tilesetindex.hpp support/mmapped/tilesetindex.cpp
  heightfunction.hpp heightfunction.cpp

  # resource definitions
  definition.hpp
  definition/factory.hpp definition/factory.cpp
  definition/options.hpp definition/options.cpp

  definition/tms.hpp
  definition/tms-common.cpp
  definition/tms-raster-synthetic.cpp
  definition/tms-raster-patchwork.cpp
  definition/tms-raster-solid.cpp

  definition/tms-raster.cpp
  definition/tms-raster-remote.cpp
  definition/tms-bing.cpp
  definition/tms-windyty.cpp

  definition/surface.hpp
  definition/surface.cpp
  definition/surface-spheroid.cpp
  definition/surface-dem.cpp

  definition/geodata.hpp
  definition/geodatavectorbase.cpp
  definition/geodata-vector.cpp
  definition/geodata-vector-tiled.cpp

  # control socket interface
  mapproxy.hpp mapproxy.cpp
  )

define_module(LIBRARY mapproxy-core
  DEPENDS
  qmf
  pysupport pydbglog
  vts-libs>=2.4 http>=1.0 geo>=1.28 geometry>=1.7
  jsoncpp>=2.1

  Boost_SERIALIZATION
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  Boost_PYTHON
  Sqlite3
  TINYXML2
  )

add_library(mapproxy-core STATIC ${mapproxy-core_SOURCES})
buildsys_library(mapproxy-core)
target_link_libraries(mapproxy-core ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-core ${MODULE_DEFINITIONS})

# ------------------------------------------------------------------------
# GDAL machinery
set(mapproxy-gdal_SOURCES
  gdalsupport.hpp gdalsupport/gdalsupport.cpp
  gdalsupport/types.hpp
  gdalsupport/requests.hpp gdalsupport/requests.cpp
  gdalsupport/process.hpp gdalsupport/process.cpp
  gdalsupport/datasetcache.hpp gdalsupport/datasetcache.cpp
  gdalsupport/operations.hpp gdalsupport/operations.cpp
  )

define_module(LIBRARY mapproxy-gdal
  DEPENDS mapproxy-core
  gdal-drivers>=1.14
  )

add_library(mapproxy-gdal STATIC ${mapproxy-gdal_SOURCES})
buildsys_library(mapproxy-gdal)
target_link_libraries(mapproxy-gdal ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-gdal ${MODULE_DEFINITIONS})

# ------------------------------------------------------------------------
# Main mapproxy binary
define_module(BINARY mapproxy
  DEPENDS
  mapproxy-gdal mapproxy-core service>=1.6

  Boost_SERIALIZATION
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  Boost_PYTHON
  )

set(mapproxy_SOURCES
  resourcebackend.hpp
  resourcebackend/resourcebackend.cpp
  resourcebackend/factory.hpp
  resourcebackend/conffile.hpp resourcebackend/conffile.cpp
  resourcebackend/python.hpp resourcebackend/python.cpp

  generator.hpp
  generator/generator.cpp
  generator/factory.hpp
  generator/metatile.hpp generator/metatile.cpp
  generator/demregistry.hpp generator/demregistry.cpp

  # bound layers
  generator/tms-raster-base.hpp generator/tms-raster-base.cpp
  generator/tms-raster.hpp generator/tms-raster.cpp
  generator/tms-raster-remote.hpp generator/tms-raster-remote.cpp
  generator/tms-bing.hpp generator/tms-bing.cpp
  generator/tms-windyty.hpp generator/tms-windyty.cpp
  generator/tms-raster-synthetic.hpp generator/tms-raster-synthetic.cpp
  generator/tms-raster-patchwork.hpp generator/tms-raster-patchwork.cpp
  generator/tms-raster-solid.hpp generator/tms-raster-solid.cpp

  # surfaces
  generator/surface.hpp generator/surface.cpp
  generator/surface-spheroid.hpp generator/surface-spheroid.cpp
  generator/surface-dem.hpp generator/surface-dem.cpp

  # free layers
  generator/geodatavectorbase.hpp generator/geodatavectorbase.cpp
  generator/geodata-vector-tiled.hpp generator/geodata-vector-tiled.cpp
  generator/geodata-vector.hpp generator/geodata-vector.cpp

  sink.hpp sink.cpp

  fileinfo.hpp fileinfo.cpp
  core.hpp core.cpp

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

set(mapproxy_FILES_SOURCES
  generator/files.hpp generator/files.cpp
  )
file_to_cpp(mapproxy_FILES_SOURCES
  files::defaultstyle_json
  generator/files/defaultstyle.json)

set(mapproxy_CESIUM_SOURCES
  cesium.hpp cesium.cpp
  )
file_to_cpp(mapproxy_CESIUM_SOURCES
  cesium::cesium_html
  cesium/cesium.html)
file_to_cpp(mapproxy_CESIUM_SOURCES
  cesium::cesium_js
  cesium/cesium.js)
file_to_cpp(mapproxy_CESIUM_SOURCES
  cesium::melowntech_png
  cesium/melown-technologies-logo-28.png)

set(mapproxy_OL_SOURCES
  ol.hpp ol.cpp
  )
file_to_cpp(mapproxy_OL_SOURCES
  ol::ol_html
  ol/ol.html)
file_to_cpp(mapproxy_OL_SOURCES
  ol::ol_js
  ol/ol.js)

add_executable(mapproxy
  ${mapproxy_SOURCES}
  ${mapproxy_BROWSER_SOURCES}
  ${mapproxy_FILES_SOURCES}
  ${mapproxy_CESIUM_SOURCES}
  ${mapproxy_OL_SOURCES}
  )
target_link_libraries(mapproxy mapproxy-core ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy)
set_target_version(mapproxy ${vts-mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy RUNTIME DESTINATION bin
  COMPONENT main)
