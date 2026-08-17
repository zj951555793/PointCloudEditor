cmake_minimum_required(VERSION 3.22.1)
project(JMEngineAndroidBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# jmengine/src/main/cpp -> JMEngine repository root
get_filename_component(JMENGINE_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/../../../../../.." ABSOLUTE)

# Android Studio builds the algorithm library only.  Desktop Qt UI, tests,
# CUDA and host helper programs are intentionally excluded from the native APK.
set(JMENGINE_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(JMENGINE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(JMENGINE_TEXTURE_DATASET_TEST OFF CACHE BOOL "" FORCE)
set(JMENGINE_BUILD_QT_EDITOR OFF CACHE BOOL "" FORCE)
set(JMENGINE_BUILD_WINDOWS_OPENGL_EXAMPLE OFF CACHE BOOL "" FORCE)
set(JMENGINE_BUILD_RK3588_GLES_EXAMPLE OFF CACHE BOOL "" FORCE)
set(JMENGINE_ENABLE_TEXTURE_MAPPING OFF CACHE BOOL "" FORCE)
set(JMENGINE_ENABLE_TEXTURE_CUDA OFF CACHE BOOL "" FORCE)
set(JMENGINE_ENABLE_OPENMP OFF CACHE BOOL "" FORCE)
set(JMENGINE_ENABLE_POISSONRECON OFF CACHE BOOL "" FORCE)

add_subdirectory("${JMENGINE_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/jmengine")

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

target_include_directories(JMEngine_android PRIVATE
    "${JMENGINE_ROOT}/include"
)

target_compile_definitions(JMEngine_android PRIVATE
    JMENGINE_ANDROID_JNI=1
)
