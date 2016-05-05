# test program
define_module(BINARY dem-tiling
  DEPENDS vts-libs service gdal-drivers geometry
  MHD
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(dem-tiling_SOURCES
  main.cpp
  )

add_executable(dem-tiling ${dem-tiling_SOURCES})
target_link_libraries(dem-tiling ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(dem-tiling ${MODULE_DEFINITIONS})
buildsys_binary(dem-tiling)
set_target_version(dem-tiling ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS dem-tiling RUNTIME DESTINATION bin
  COMPONENT main)
