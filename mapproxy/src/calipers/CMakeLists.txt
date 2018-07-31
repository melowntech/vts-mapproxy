# test program
define_module(BINARY mp-calipers
  DEPENDS vts-libs service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mp-calipers_SOURCES
  calipers.hpp calipers.cpp
  )

add_library(mp-calipers ${mp-calipers_SOURCES})
target_link_libraries(mp-calipers ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mp-calipers ${MODULE_DEFINITIONS})
buildsys_library(mp-calipers)

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# test program
define_module(BINARY mapproxy-calipers
  DEPENDS vts-libs service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mapproxy-calipers_SOURCES
  main.cpp
  )

add_executable(mapproxy-calipers ${mapproxy-calipers_SOURCES})
target_link_libraries(mapproxy-calipers mp-calipers ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-calipers ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-calipers)
set_target_version(mapproxy-calipers ${vts-mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy-calipers RUNTIME DESTINATION bin
  COMPONENT tools)
