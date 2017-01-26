# test program
define_module(BINARY generatevrtwo
  DEPENDS service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(generatevrtwo_SOURCES
  main.cpp
  )

add_executable(generatevrtwo ${generatevrtwo_SOURCES})
target_link_libraries(generatevrtwo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(generatevrtwo ${MODULE_DEFINITIONS})
buildsys_binary(generatevrtwo)
set_target_version(generatevrtwo ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS generatevrtwo RUNTIME DESTINATION bin
  COMPONENT tools)
