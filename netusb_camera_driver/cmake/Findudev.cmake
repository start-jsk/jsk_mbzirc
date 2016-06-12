# Variables
# UDEV_INCLUDE_DIRS
# UDEV_FOUND
# UDEV_LIBRARIES

set(UDEV_ROOT_DIR
  "${UDEV_ROOT_DIR}"
  CACHE PATH
  "Directory to search for udev")

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_LIBUDEV libudev)
endif()

find_library(UDEV_LIBRARY
  NAMES udev
  PATHS ${PC_LIBUDEV_LIBRARY_DIRS} ${PC_LIBUDEV_LIBDIR}
  HINTS "${UDEV_ROOT_DIR}"
  PATH_SUFFIXES lib)

get_filename_component(_libdir "${UDEV_LIBRARY}" PATH)

find_path(UDEV_INCLUDE_DIR
  NAMES libudev.h
  PATHS ${PC_LIBUDEV_INCLUDE_DIRS} ${PC_LIBUDEV_INCLUDEDIR}
  HINTS "${_libdir}" "${_libdir}/.." "${UDEV_ROOT_DIR}"
  PATH_SUFFIXES include)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(UDEV
  DEFAULT_MSG UDEV_LIBRARY UDEV_INCLUDE_DIR)

if(UDEV_FOUND)
  list(APPEND UDEV_LIBRARIES ${UDEV_LIBRARY})
  list(APPEND UDEV_INCLUDE_DIRS ${UDEV_INCLUDE_DIR})
  mark_as_advanced(UDEV_ROOT_DIR)
endif()

mark_as_advanced(UDEV_INCLUDE_DIR UDEV_LIBRARY)

# patch for version mismatch
function(_patch_udev_version_mismatch)
  get_filename_component(UDEV_LIBRARY_DIR ${UDEV_LIBRARY} DIRECTORY)
  set(UDEV_LIB_DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libudev.so.0")
  if (EXISTS "${UDEV_LIBRARY_DIR}/libudev.so.0")
    message(STATUS "Found dynamic library at ${UDEV_LIBRARY_DIR}/libudev.so.0")
  elseif(EXISTS ${UDEV_LIB_DESTINATION})
    message(STATUS "Found dynamic library at ${UDEV_LIB_DESTINATION}")
  else()
    message(STATUS "Not found dynamic library libudev.so.0")
    execute_process(
      COMMAND ln -sf ${UDEV_LIBRARY} ${UDEV_LIB_DESTINATION}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      OUTPUT_VARIABLE CREATE_SYMLINK_OUTPUT
      ERROR_VARIABLE CREATE_SYMLINK_ERROR)
    if(EXISTS ${UDEV_LIB_DESTINATION})
      message(STATUS "Created symlink to: ${UDEV_LIB_DESTINATION}")
    else()
      message(WARNING "Failed to create symlink: ${CREATE_SYMLINK_ERROR}")
    endif()
  endif()
endfunction()
_patch_udev_version_mismatch()
