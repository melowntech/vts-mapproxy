# test program
define_module(BINARY mp-tiling
  DEPENDS vts-libs service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mp-tiling_SOURCES
  tiling.hpp tiling.cpp
  )

add_library(mp-tiling ${mp-tiling_SOURCES})
target_link_libraries(mp-tiling ${MODULE_LIBRARIES})
target_compile_definitions(mp-tiling PRIVATE ${MODULE_DEFINITIONS})
buildsys_library(mp-tiling)

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# test program
define_module(BINARY mapproxy-tiling
  DEPENDS vts-libs service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mapproxy-tiling_SOURCES
  main.cpp
  )

add_executable(mapproxy-tiling ${mapproxy-tiling_SOURCES})
target_link_libraries(mapproxy-tiling mp-tiling ${MODULE_LIBRARIES})
target_compile_definitions(mapproxy-tiling PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-tiling)
set_target_version(mapproxy-tiling ${vts-mapproxy_VERSION})

set(sub-tiling_SOURCES
  sub-tiling.cpp
  )

add_executable(mapproxy-sub-tiling ${sub-tiling_SOURCES})
target_link_libraries(mapproxy-sub-tiling mp-tiling ${MODULE_LIBRARIES})
target_compile_definitions(mapproxy-sub-tiling PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-sub-tiling)
set_target_version(mapproxy-sub-tiling ${vts-mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy-tiling RUNTIME DESTINATION bin
  COMPONENT tools)
