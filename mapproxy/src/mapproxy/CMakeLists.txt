# test program
define_module(BINARY mapproxy
  DEPENDS vts-libs service)

set(mapproxy_SOURCES
  main.cpp
  )

add_executable(mapproxy ${mapproxy_SOURCES})
target_link_libraries(mapproxy ${MODULE_LIBRARIES})
buildsys_binary(mapproxy)
set_target_version(mapproxy ${mapproxy_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS mapproxy RUNTIME DESTINATION bin
  COMPONENT main)
