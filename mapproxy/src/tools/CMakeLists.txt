# tools
define_module(BINARY tools
  DEPENDS mapproxy-core service

  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS
  )

# ----------------------------------------------------------------------
# vts::TileIndex to mapproxy's mapped::TileIndex
set(mapproxy-ti2mmti_SOURCES
  ti2mmti.cpp
  )

add_executable(mapproxy-ti2mmti ${mapproxy-ti2mmti_SOURCES})
target_link_libraries(mapproxy-ti2mmti ${MODULE_LIBRARIES})
target_compile_definitions(mapproxy-ti2mmti PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-ti2mmti)
set_target_version(mapproxy-ti2mmti ${vts-mapproxy_VERSION})

# ----------------------------------------------------------------------
# vts::TileIndex and mapped::TileIndex query tool
set(mapproxy-querymmti_SOURCES
  querymmti.cpp
  )

add_executable(mapproxy-querymmti ${mapproxy-querymmti_SOURCES})
target_link_libraries(mapproxy-querymmti ${MODULE_LIBRARIES})
target_compile_definitions(mapproxy-querymmti PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-querymmti)
set_target_version(mapproxy-querymmti ${vts-mapproxy_VERSION})
