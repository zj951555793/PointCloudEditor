set(rulermvs_VERSION ${PACKAGE_PREFIX_DIR}/1.4.30)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include("${CMAKE_CURRENT_LIST_DIR}/rulermvsTargets.cmake")
set_and_check(rulermvs_LIB_DIR "${PACKAGE_PREFIX_DIR}/lib")
set_and_check(rulermvs_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")

message(STATUS "rulermvs libraries version: ${rulermvs_VERSION}")
message(STATUS "rulermvs libraries location: ${rulermvs_LIB_DIR}")

check_required_components(rulermvs_core;rulermvs_match;rulermvs_phaseshift;rulermvs_rgbdfusion;rulermvs_FaceScan;rulermvs_Lines_MarkerFusion;rulermvs_MarkerExtractor;rulermvs_OralScan;rulermvs_RGBD_MarkerFusion;rulermvs_Tracker;rulermvs_multiframefilter;rulermvs_multilines;rulermvs_oneshot;rulermvs_rgbslam)

set(rulermvs_FOUND TRUE)
set(rulermvs_LIBRARIES rulermvs_core;rulermvs_match;rulermvs_phaseshift;rulermvs_rgbdfusion;rulermvs_FaceScan;rulermvs_Lines_MarkerFusion;rulermvs_MarkerExtractor;rulermvs_OralScan;rulermvs_RGBD_MarkerFusion;rulermvs_Tracker;rulermvs_multiframefilter;rulermvs_multilines;rulermvs_oneshot;rulermvs_rgbslam)
set(rulermvs_INCLUDE_DIRS ${rulermvs_INCLUDE_DIR})
