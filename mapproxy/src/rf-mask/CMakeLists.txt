# test program
define_module(BINARY rf-mask
  DEPENDS vts-libs imgproc service gdal-drivers geometry
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  Boost_REGEX)

set(rf-mask_SOURCES
  main.cpp
  ogrsupport.hpp ogrsupport.cpp
  )

add_executable(mapproxy-rf-mask ${rf-mask_SOURCES})
target_link_libraries(mapproxy-rf-mask ${MODULE_LIBRARIES})
target_compile_definitions(mapproxy-rf-mask PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-rf-mask)
set_target_version(mapproxy-rf-mask ${vts-mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy-rf-mask RUNTIME DESTINATION bin
  COMPONENT tools)
