# test program
define_module(BINARY mapproxy
  DEPENDS vts-libs service
  MHD
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mapproxy_SOURCES
  error.hpp
  resource.hpp resource.cpp

  resourcebackend.hpp
  resourcebackend/resourcebackend.cpp
  resourcebackend/factory.hpp
  resourcebackend/conffile.hpp resourcebackend/conffile.cpp
  resourcebackend/mysql.hpp resourcebackend/mysql.cpp

  generator.hpp
  generator/generator.cpp
  generator/factory.hpp
  generator/tms-raster.hpp generator/tms-raster.cpp

  http.hpp http.cpp
  contentgenerator.hpp

  fileinfo.hpp fileinfo.cpp
  core.hpp core.cpp

  main.cpp
  )

add_executable(mapproxy ${mapproxy_SOURCES})
target_link_libraries(mapproxy ${MODULE_LIBRARIES})
buildsys_binary(mapproxy)
set_target_version(mapproxy ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy RUNTIME DESTINATION bin
  COMPONENT main)
