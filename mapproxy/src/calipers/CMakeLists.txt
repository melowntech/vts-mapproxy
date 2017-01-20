# test program
define_module(BINARY mapproxy-calipers
  DEPENDS service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mapproxy-calipers_SOURCES
  main.cpp
  )

add_executable(mapproxy-calipers ${mapproxy-calipers_SOURCES})
target_link_libraries(mapproxy-calipers ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-calipers ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-calipers)
set_target_version(mapproxy-calipers ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy-calipers RUNTIME DESTINATION bin
  COMPONENT main)
