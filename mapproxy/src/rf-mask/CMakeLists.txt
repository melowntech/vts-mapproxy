# test program
define_module(BINARY rf-mask
  DEPENDS vts-libs imgproc service gdal-drivers geometry
  MHD
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(rf-mask_SOURCES
  main.cpp
  ogrsupport.hpp ogrsupport.cpp
  )

add_executable(rf-mask ${rf-mask_SOURCES})
target_link_libraries(rf-mask ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(rf-mask ${MODULE_DEFINITIONS})
buildsys_binary(rf-mask)
set_target_version(rf-mask ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS rf-mask RUNTIME DESTINATION bin
  COMPONENT main)
