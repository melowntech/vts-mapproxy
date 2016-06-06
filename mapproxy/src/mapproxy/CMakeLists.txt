# test program
define_module(BINARY mapproxy
  DEPENDS vts-libs service gdal-drivers geometry
  MHD
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mapproxy_SOURCES
  error.hpp
  resource.hpp resource.cpp
  support/metatile.hpp support/metatile.cpp
  support/mesh.hpp support/mesh.cpp
  support/geo.hpp support/geo.cpp
  support/coverage.hpp support/coverage.cpp

  resourcebackend.hpp
  resourcebackend/resourcebackend.cpp
  resourcebackend/factory.hpp
  resourcebackend/conffile.hpp resourcebackend/conffile.cpp
  resourcebackend/mysql.hpp resourcebackend/mysql.cpp

  generator.hpp
  generator/generator.cpp
  generator/factory.hpp
  generator/tms-raster.hpp generator/tms-raster.cpp

  generator/surface.hpp generator/surface.cpp
  generator/surface-spheroid.hpp generator/surface-spheroid.cpp
  generator/surface-dem.hpp generator/surface-dem.cpp

  http.hpp http.cpp
  contentgenerator.hpp contentgenerator.cpp

  fileinfo.hpp fileinfo.cpp
  core.hpp core.cpp

  gdalsupport.hpp gdalsupport/gdalsupport.cpp
  gdalsupport/types.hpp
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
  browser2d::leaflet_js
  browser2d/leaflet.js)
file_to_cpp(mapproxy_BROWSER_SOURCES
  browser2d::leaflet_css
  browser2d/leaflet.css)

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
