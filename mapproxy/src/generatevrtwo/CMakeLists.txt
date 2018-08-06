# test program
define_module(BINARY mp-generatevrtwo
  DEPENDS vts-libs service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mp-generatevrtwo_SOURCES
  generatevrtwo.hpp generatevrtwo.cpp
  )

add_library(mp-generatevrtwo ${mp-generatevrtwo_SOURCES})
target_link_libraries(mp-generatevrtwo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mp-generatevrtwo ${MODULE_DEFINITIONS})
buildsys_library(mp-generatevrtwo)

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# test program
define_module(BINARY generatevrtwo
  DEPENDS vts-libs service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(generatevrtwo_SOURCES
  main.cpp
  )

add_executable(generatevrtwo ${generatevrtwo_SOURCES})
target_link_libraries(generatevrtwo mp-generatevrtwo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(generatevrtwo ${MODULE_DEFINITIONS})
buildsys_binary(generatevrtwo)
set_target_version(generatevrtwo ${vts-mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS generatevrtwo RUNTIME DESTINATION bin
  COMPONENT tools)
