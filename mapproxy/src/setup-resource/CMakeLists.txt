# test program
define_module(BINARY setup-resource
  DEPENDS mp-generatevrtwo mp-calipers mapproxy-core
  vts-libs imgproc service gdal-drivers geometry
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  Boost_REGEX)

set(setup-resource_SOURCES
  main.cpp
  )

add_executable(mapproxy-setup-resource ${setup-resource_SOURCES})
target_link_libraries(mapproxy-setup-resource ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-setup-resource ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-setup-resource)
set_target_version(mapproxy-setup-resource ${vts-mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy-setup-resource RUNTIME DESTINATION bin
  COMPONENT tools)
