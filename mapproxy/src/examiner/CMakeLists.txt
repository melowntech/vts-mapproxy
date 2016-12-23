# test program
define_module(BINARY mapproxy-examiner
  DEPENDS service gdal-drivers geometry geo
  Boost_FILESYSTEM
  Boost_PROGRAM_OPTIONS)

set(mapproxy-examiner_SOURCES
  main.cpp
  )

add_executable(mapproxy-examiner ${mapproxy-examiner_SOURCES})
target_link_libraries(mapproxy-examiner ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(mapproxy-examiner ${MODULE_DEFINITIONS})
buildsys_binary(mapproxy-examiner)
set_target_version(mapproxy-examiner ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy-examiner RUNTIME DESTINATION bin
  COMPONENT main)
