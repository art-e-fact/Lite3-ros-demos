# Post-install hook: symlink drdds typesupport dylibs into the active
# conda/pixi env's lib/ directory so that macOS binaries
# can dlopen them via dyld's default search path even
# when DYLD_LIBRARY_PATH is stripped on exec. See drdds/CMakeLists.txt.

set(_conda_prefix "$ENV{CONDA_PREFIX}")
if(NOT _conda_prefix)
  set(_conda_prefix "$ENV{PIXI_PROJECT_ROOT}/.pixi/envs/default")
endif()

if(NOT EXISTS "${_conda_prefix}/lib")
  message(WARNING "drdds: CONDA_PREFIX lib dir not found at ${_conda_prefix}/lib; mjpython may fail to dlopen drdds typesupport dylibs on macOS.")
  return()
endif()

file(GLOB _drdds_dylibs "${CMAKE_INSTALL_PREFIX}/lib/libdrdds*.dylib")
foreach(_lib IN LISTS _drdds_dylibs)
  get_filename_component(_name "${_lib}" NAME)
  set(_dest "${_conda_prefix}/lib/${_name}")
  message(STATUS "drdds: symlinking ${_dest} -> ${_lib}")
  file(REMOVE "${_dest}")
  file(CREATE_LINK "${_lib}" "${_dest}" SYMBOLIC)
endforeach()
