cmake_minimum_required(VERSION 3.22.1)
project(JMEngineAndroidBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Locate the JMEngine repository root instead of relying on a fixed number of
# "../" segments. This remains valid when Android Studio copies or relocates
# the Gradle project under another directory name.
set(JMENGINE_ROOT "" CACHE PATH "JMEngine repository root")
if(NOT JMENGINE_ROOT)
    set(_jmengine_candidate "${CMAKE_CURRENT_LIST_DIR}")
    foreach(_level RANGE 0 10)
        if(EXISTS "${_jmengine_candidate}/include/JMEngine/JMEngine.h"
           AND EXISTS "${_jmengine_candidate}/src/JMEngine.cpp")
            set(JMENGINE_ROOT "${_jmengine_candidate}")
            break()
        endif()
        get_filename_component(_jmengine_candidate "${_jmengine_candidate}/.." ABSOLUTE)
    endforeach()
endif()
if(NOT JMENGINE_ROOT
   OR NOT EXISTS "${JMENGINE_ROOT}/include/JMEngine/JMEngine.h"
   OR NOT EXISTS "${JMENGINE_ROOT}/src/JMEngine.cpp")
    message(FATAL_ERROR
        "Cannot locate JMEngine root. Open android/native_app from the complete "
        "JMEngine source tree, or set -DJMENGINE_ROOT=<directory containing include/ and src/>.")
endif()
message(STATUS "JMEngine root: ${JMENGINE_ROOT}")

# Build the Android-facing JMEngine core directly. This avoids nesting the root
# project with add_subdirectory(), so Android Studio no longer depends on a
# copied/top-level CMakeLists.txt or a second CMake binary directory.
add_library(JMEngine STATIC
    "${JMENGINE_ROOT}/src/PointCloud.cpp"
    "${JMENGINE_ROOT}/src/JMEngine.cpp"
    "${JMENGINE_ROOT}/src/PointCloudIO.cpp"
    "${JMENGINE_ROOT}/src/scanner/FrameQueue.cpp"
    "${JMENGINE_ROOT}/src/scanner/JMScanner.cpp"
    "${JMENGINE_ROOT}/src/scanner/RulerMvsSlam.cpp"
    "${JMENGINE_ROOT}/src/scanner/CameraSource.cpp"
)
target_include_directories(JMEngine PUBLIC "${JMENGINE_ROOT}/include")
target_compile_features(JMEngine PUBLIC cxx_std_17)

# Optional Android RulerMVS SDK layout:
#   <root>/include/{rulermvs.hpp,DBoW3/...}
#   <root>/lib/arm64-v8a/*.so
#   <root>/opencv/sdk/native/{jni/include,libs/arm64-v8a/*.so}
set(JMENGINE_ANDROID_RULERMVS_ROOT "" CACHE PATH "Android RulerMVS SDK root")
if(JMENGINE_ANDROID_RULERMVS_ROOT)
    set(_ruler_inc "${JMENGINE_ANDROID_RULERMVS_ROOT}/include")
    set(_opencv_inc "${JMENGINE_ANDROID_RULERMVS_ROOT}/opencv/sdk/native/jni/include")
    file(GLOB _ruler_so CONFIGURE_DEPENDS "${JMENGINE_ANDROID_RULERMVS_ROOT}/lib/${ANDROID_ABI}/*.so")
    file(GLOB _opencv_so CONFIGURE_DEPENDS "${JMENGINE_ANDROID_RULERMVS_ROOT}/opencv/sdk/native/libs/${ANDROID_ABI}/*.so")
    if(NOT EXISTS "${_ruler_inc}/rulermvs.hpp" OR NOT _ruler_so)
        message(FATAL_ERROR "Incomplete Android RulerMVS SDK: ${JMENGINE_ANDROID_RULERMVS_ROOT}")
    endif()
    target_include_directories(JMEngine PRIVATE "${_ruler_inc}" "${_opencv_inc}")
    target_compile_definitions(JMEngine PRIVATE JMENGINE_HAS_RULERMVS=1)
    target_link_libraries(JMEngine PRIVATE ${_ruler_so} ${_opencv_so})
endif()

add_library(JMEngine_android SHARED
    JMEngineAndroidJni.cpp
    JMEngineGlesRenderer.cpp
)

target_link_libraries(JMEngine_android PRIVATE
    JMEngine
    android
    log
    GLESv3
)

if (ANDROID)
    target_link_options(JMEngine_android PRIVATE "-Wl,-z,max-page-size=16384")
endif()

target_include_directories(JMEngine_android PRIVATE
    "${JMENGINE_ROOT}/include"
)

target_compile_definitions(JMEngine_android PRIVATE
    JMENGINE_ANDROID_JNI=1
)
