# test program
define_module(BINARY dem-tiling
  DEPENDS vts-libs service gdal-drivers geometry
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(dem-tiling_SOURCES
  main.cpp
  )

add_executable(mapproxy-dem-tiling ${dem-tiling_SOURCES})
target_link_libraries(mapproxy-dem-tiling ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-dem-tiling ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-dem-tiling)
set_target_version(mapproxy-dem-tiling ${mapproxy_VERSION})

set(sub-tiling_SOURCES
  sub-tiling.cpp
  )

add_executable(mapproxy-sub-tiling ${sub-tiling_SOURCES})
target_link_libraries(mapproxy-sub-tiling ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-sub-tiling ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-sub-tiling)
set_target_version(mapproxy-sub-tiling ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy-dem-tiling RUNTIME DESTINATION bin
  COMPONENT main)
