# test program
define_module(BINARY addresource
  DEPENDS vts-libs imgproc service gdal-drivers geometry
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  Boost_REGEX)

set(addresource_SOURCES
  main.cpp
  )

add_executable(mapproxy-addresource ${addresource_SOURCES})
target_link_libraries(mapproxy-addresource ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-addresource ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-addresource)
set_target_version(mapproxy-addresource ${vts-mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy-addresource RUNTIME DESTINATION bin
  COMPONENT tools)
