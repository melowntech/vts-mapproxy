# test program
define_module(BINARY generate-tileindex
  DEPENDS mapproxy-core
  vts-libs imgproc service gdal-drivers geometry
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  Boost_REGEX)

set(generate-tileindex_SOURCES
  generate-tileindex.cpp
  )

add_executable(mapproxy-generate-tileindex ${generate-tileindex_SOURCES})
target_link_libraries(mapproxy-generate-tileindex ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-generate-tileindex ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-generate-tileindex)
set_target_version(mapproxy-generate-tileindex ${vts-mapproxy_VERSION})
