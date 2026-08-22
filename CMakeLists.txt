cmake_minimum_required(VERSION 3.16)
project(JMEngine VERSION 2.4.0 LANGUAGES CXX)

# Visual Studio / Xcode 等 IDE 中启用目录分组。
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_CUDA_ARCHITECTURES 50)
# PoissonRecon / large template-heavy translation units can exceed the default
# COFF section limit under MSVC (C1128). Apply /bigobj to every MSVC target so
# the core library, Qt editor, tests, and examples all use the extended object
# format consistently. This is a compile-time format change only.
if(MSVC)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/bigobj>)
endif()

option(JMENGINE_BUILD_EXAMPLES "Build basic example" ON)
option(JMENGINE_BUILD_TESTS "Build core tests" ON)
option(JMENGINE_TEXTURE_DATASET_TEST "Enable auto-download BOP LINEMOD Mesh->Texture integration test" ON)
option(JMENGINE_BUILD_QT_EDITOR "Build the single unified Qt editor" ON)
set(JMENGINE_RENDER_BACKEND "AUTO" CACHE STRING "Qt renderer backend: AUTO, DESKTOP_GL21, GLES31")
set_property(CACHE JMENGINE_RENDER_BACKEND PROPERTY STRINGS AUTO DESKTOP_GL21 GLES31)
option(JMENGINE_BUILD_WINDOWS_OPENGL_EXAMPLE "Build native Win32/WGL backend test" OFF)
option(JMENGINE_BUILD_RK3588_GLES_EXAMPLE "Build native EGL/GLES backend test" OFF)
option(JMENGINE_ENABLE_OPENMP "Enable OpenMP acceleration when available" ON)
option(JMENGINE_ENABLE_TEXTURE_MAPPING "Build CPU/CUDA texture mapping library" ON)
option(JMENGINE_ENABLE_TEXTURE_CUDA "Enable optional CUDA backend for texture mapping when a CUDA compiler is available" ON)
option(JMENGINE_ENABLE_POISSONRECON "Enable industrial Kazhdan Poisson reconstruction" ON)
option(JMENGINE_FETCH_POISSONRECON "Developer fallback: fetch PoissonRecon 18.76 when no bundled/local source exists" OFF)
option(JMENGINE_REQUIRE_POISSONRECON "Fail configure when the industrial Poisson backend is unavailable" OFF)
option(JMENGINE_POISSON_BIG_DATA "Enable PoissonRecon BIG_DATA indexing" OFF)
set(JMENGINE_POISSONRECON_ROOT "" CACHE PATH "Optional external PoissonRecon source tree (directory containing Src/Reconstructors.h)")
set(JMENGINE_POISSONRECON_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/PoissonRecon" CACHE PATH "Bundled/vendor PoissonRecon 18.76 source tree")

# -----------------------------
# Android / RK3588 GLES platform policy
# -----------------------------
# Android V12 focuses on getting the existing Qt editor running unchanged on arm64-v8a.
# Rendering uses the same GLES 3.1 backend as RK3588 Linux. CUDA/Win32-only integrations
# are disabled automatically. PoissonRecon's current official SurfaceTrimmer integration
# launches a helper executable, which is not an Android-app-safe deployment model, so the
# industrial Poisson backend is disabled on Android until the trimmer is moved in-process.
if(ANDROID)
    set(JMENGINE_RENDER_BACKEND "GLES31" CACHE STRING "Qt renderer backend" FORCE)
    set(JMENGINE_ENABLE_TEXTURE_CUDA OFF CACHE BOOL "CUDA is unavailable in the Android NDK build" FORCE)
    set(JMENGINE_BUILD_WINDOWS_OPENGL_EXAMPLE OFF CACHE BOOL "Win32 example is unavailable on Android" FORCE)
    set(JMENGINE_BUILD_RK3588_GLES_EXAMPLE OFF CACHE BOOL "Native EGL example is not part of the Android APK" FORCE)
    set(JMENGINE_BUILD_TESTS OFF CACHE BOOL "Host CTest binaries are not packaged into the Android APK" FORCE)
    set(JMENGINE_TEXTURE_DATASET_TEST OFF CACHE BOOL "Dataset test is host-side only" FORCE)
    set(JMENGINE_BUILD_EXAMPLES OFF CACHE BOOL "CLI example is host-side only" FORCE)
    set(JMENGINE_ENABLE_POISSONRECON OFF CACHE BOOL "Android V12: Poisson SurfaceTrimmer helper is not packaged" FORCE)
    add_compile_definitions(JMENGINE_PLATFORM_ANDROID=1)
    message(STATUS "JMEngine Android policy: arm64 Qt/Widgets + GLES3.1; CUDA/Poisson helper disabled")
endif()

# OBJ/PLY 大模型的 CPU 后处理可使用 OpenMP 并行。
# OpenMP 为可选依赖：找不到时仍然使用串行实现，保证库/示例可正常构建。
if(JMENGINE_ENABLE_OPENMP)
    find_package(OpenMP QUIET)
endif()

# 所有公开头文件都显式加入 target_sources。
# 这样使用 Visual Studio CMake Generator 生成 .sln/.vcxproj 时，Header Files 下会显示 .h 文件，
# 不会出现“库能编译，但 VS 项目树里只有 .cpp 没有 .h”的问题。
set(JMENGINE_PUBLIC_HEADERS
    include/JMEngine/Types.h
    include/JMEngine/Alignment.h
    include/JMEngine/Measurement.h
    include/JMEngine/Edit.h
    include/JMEngine/PointCloud.h
    include/JMEngine/Selection.h
    include/JMEngine/JMEngine.h
    include/JMEngine/ScanTypes.h
    include/JMEngine/ISlam.h
    include/JMEngine/FrameQueue.h
    include/JMEngine/JMScanner.h
    include/JMEngine/RulerMvsSlam.h
    include/JMEngine/ScanProject.h
    include/JMEngine/ProjectManager.h
    include/JMEngine/CameraSource.h
    include/JMEngine/TriangleMesh.h
    include/JMEngine/edit/DirtyRange.h
    include/JMEngine/edit/EditResult.h
    include/JMEngine/edit/PointCloudEditSession.h
    include/JMEngine/edit/MeshEditSession.h
    include/JMEngine/CpuSelector.h
    include/JMEngine/CpuMeshSelector.h
    include/JMEngine/MeshSelectionClosure.h
    include/JMEngine/IGpuPicker.h
    include/JMEngine/PixelIdPicker.h
    include/JMEngine/ColorPicking24.h
    include/JMEngine/BlockPicking24.h
    include/JMEngine/OpenGLPickingShaders.h
    include/JMEngine/PointCloudIO.h
    include/JMEngine/ObjMeshLoader.h
    include/JMEngine/ObjAppearanceLoader.h
    include/JMEngine/ObjModelLoader.h
    include/JMEngine/MeshUtils.h
    include/JMEngine/ModelIO.h
    include/JMEngine/processing/Processing.h
    include/JMEngine/processing/Parallel.h
    include/JMEngine/processing/Operations.h
    include/JMEngine/processing/Diagnostics.h
)

set(JMENGINE_SOURCES
    src/PointCloud.cpp
    src/Alignment.cpp
    src/Measurement.cpp
    src/JMEngine.cpp
    src/scanner/JMScanner.cpp
    src/scanner/RulerMvsSlam.cpp
    src/scanner/ScanProject.cpp
    src/scanner/ProjectManager.cpp
    src/scanner/CameraSource.cpp
    src/scanner/FrameQueue.cpp
    src/TriangleMesh.cpp
    src/edit/DirtyRange.cpp
    src/edit/MeshEditSession.cpp
    src/CpuSelector.cpp
    src/CpuMeshSelector.cpp
    src/MeshSelectionClosure.cpp
    src/PointCloudIO.cpp
    src/ObjMeshLoader.cpp
    src/ObjAppearanceLoader.cpp
    src/ObjModelLoader.cpp
    src/MeshUtils.cpp
    src/ModelIO.cpp
    src/processing/Processing.cpp
    src/processing/Diagnostics.cpp
    src/processing/Operations.cpp
    src/processing/IndustrialPoisson.cpp
)


# -----------------------------
# Industrial PoissonRecon dependency
# -----------------------------
# Production policy:
#   1) Prefer the repository-vendored third_party/PoissonRecon tree.
#   2) Allow an explicit external JMENGINE_POISSONRECON_ROOT override.
#   3) Network FetchContent is developer-only and OFF by default.
# This keeps cmake-gui / CI / factory machines reproducible and offline by default.
set(_JMENGINE_POISSON_SRC_DIR "")
set(_JMENGINE_POISSON_SOURCE_ROOT "")
if(JMENGINE_ENABLE_POISSONRECON)
    if(JMENGINE_POISSONRECON_ROOT)
        set(_JMENGINE_POISSON_SOURCE_ROOT "${JMENGINE_POISSONRECON_ROOT}")
    elseif(EXISTS "${JMENGINE_POISSONRECON_VENDOR_DIR}/Src/Reconstructors.h")
        set(_JMENGINE_POISSON_SOURCE_ROOT "${JMENGINE_POISSONRECON_VENDOR_DIR}")
    elseif(JMENGINE_FETCH_POISSONRECON)
        include(FetchContent)
        FetchContent_Declare(
            poissonrecon1876
            URL "https://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version18.76/AdaptiveSolvers.zip"
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
        FetchContent_GetProperties(poissonrecon1876)
        if(NOT poissonrecon1876_POPULATED)
            FetchContent_Populate(poissonrecon1876)
        endif()
        file(GLOB_RECURSE _JMENGINE_POISSON_RECONSTRUCTORS
            "${poissonrecon1876_SOURCE_DIR}/Reconstructors.h"
            "${poissonrecon1876_SOURCE_DIR}/*/Reconstructors.h")
        list(LENGTH _JMENGINE_POISSON_RECONSTRUCTORS _JMENGINE_POISSON_HEADER_COUNT)
        if(_JMENGINE_POISSON_HEADER_COUNT GREATER 0)
            list(GET _JMENGINE_POISSON_RECONSTRUCTORS 0 _JMENGINE_POISSON_RECONSTRUCTORS_H)
            get_filename_component(_JMENGINE_POISSON_SRC_DIR "${_JMENGINE_POISSON_RECONSTRUCTORS_H}" DIRECTORY)
            get_filename_component(_JMENGINE_POISSON_SOURCE_ROOT "${_JMENGINE_POISSON_SRC_DIR}" DIRECTORY)
        else()
            message(FATAL_ERROR "Fetched PoissonRecon 18.76 archive does not contain Reconstructors.h")
        endif()
    endif()

    if(_JMENGINE_POISSON_SOURCE_ROOT AND NOT _JMENGINE_POISSON_SRC_DIR)
        if(EXISTS "${_JMENGINE_POISSON_SOURCE_ROOT}/Src/Reconstructors.h")
            set(_JMENGINE_POISSON_SRC_DIR "${_JMENGINE_POISSON_SOURCE_ROOT}/Src")
        elseif(EXISTS "${_JMENGINE_POISSON_SOURCE_ROOT}/Reconstructors.h")
            set(_JMENGINE_POISSON_SRC_DIR "${_JMENGINE_POISSON_SOURCE_ROOT}")
        else()
            message(FATAL_ERROR "PoissonRecon root does not contain Src/Reconstructors.h: ${_JMENGINE_POISSON_SOURCE_ROOT}")
        endif()
    endif()

    # Guard against accidentally vendoring a different upstream revision.
    if(_JMENGINE_POISSON_SOURCE_ROOT AND EXISTS "${_JMENGINE_POISSON_SOURCE_ROOT}/README.md")
        file(READ "${_JMENGINE_POISSON_SOURCE_ROOT}/README.md" _JMENGINE_POISSON_README LIMIT 65536)
        string(FIND "${_JMENGINE_POISSON_README}" "Version 18.76" _JMENGINE_POISSON_VERSION_POS)
        if(_JMENGINE_POISSON_VERSION_POS EQUAL -1)
            message(WARNING "PoissonRecon source does not advertise Version 18.76; verify third_party/PoissonRecon before production release")
        endif()
    endif()

    if(NOT _JMENGINE_POISSON_SRC_DIR AND JMENGINE_REQUIRE_POISSONRECON)
        message(FATAL_ERROR
            "Industrial PoissonRecon 18.76 is required but not vendored. "
            "Run: python tools/vendor_poissonrecon.py  (or pass --archive <AdaptiveSolvers.zip>)")
    endif()
endif()

# The upstream ThreadPool stores hardware_concurrency() in a private static and uses it explicitly
# in OpenMP num_threads(). To honor JMEngine's CPU-1 policy we copy the header-only source
# into the build tree and add one setter. No solver/FEM code is changed.
if(_JMENGINE_POISSON_SRC_DIR)
    set(_JMENGINE_POISSON_PATCHED_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/poissonrecon1876-patched/Src")
    file(REMOVE_RECURSE "${_JMENGINE_POISSON_PATCHED_DIR}")
    file(MAKE_DIRECTORY "${_JMENGINE_POISSON_PATCHED_DIR}")
    file(COPY "${_JMENGINE_POISSON_SRC_DIR}/" DESTINATION "${_JMENGINE_POISSON_PATCHED_DIR}")
    set(_JMENGINE_POISSON_MT "${_JMENGINE_POISSON_PATCHED_DIR}/MultiThreading.h")
    file(READ "${_JMENGINE_POISSON_MT}" _JMENGINE_POISSON_MT_TEXT)
    string(FIND "${_JMENGINE_POISSON_MT_TEXT}" "SetNumThreads" _JMENGINE_POISSON_HAS_SETTER)
    if(_JMENGINE_POISSON_HAS_SETTER EQUAL -1)
        string(REPLACE
            "static unsigned int NumThreads( void ){ return _NumThreads; }"
            "static unsigned int NumThreads( void ){ return _NumThreads; }\n\t\tstatic void SetNumThreads( unsigned int n ){ _NumThreads = n ? n : 1; }"
            _JMENGINE_POISSON_MT_TEXT
            "${_JMENGINE_POISSON_MT_TEXT}")
        file(WRITE "${_JMENGINE_POISSON_MT}" "${_JMENGINE_POISSON_MT_TEXT}")
        string(FIND "${_JMENGINE_POISSON_MT_TEXT}" "SetNumThreads" _JMENGINE_POISSON_PATCH_OK)
        if(_JMENGINE_POISSON_PATCH_OK EQUAL -1)
            message(FATAL_ERROR "PoissonRecon MultiThreading.h layout changed; CPU-1 integration patch could not be applied")
        endif()
    endif()
    message(STATUS "JMEngine: industrial PoissonRecon backend enabled (${_JMENGINE_POISSON_SRC_DIR})")
else()
    message(STATUS "JMEngine: PoissonRecon 18.76 not vendored; core/editor still build, reconstruction reports backend unavailable")
endif()

add_library(JMEngine STATIC)
# 注意：公开头文件只是“安装/包含接口”，不要作为 PUBLIC source 传播给依赖目标。
# 如果写成 PUBLIC，Visual Studio 会把 JMEngine 的 .h 同时显示到 Examples 项目下，
# 看起来像示例工程“公开加入了库头文件”。这里改成 PRIVATE 后：
# - JMEngine 项目自身仍能在 Header Files 下看到所有头文件；
# - JMEngine_example / Qt / Windows 示例不会重复出现这些头文件；
# - target_include_directories(PUBLIC ...) 仍然保证 #include <JMEngine/...> 正常工作。
target_sources(JMEngine PRIVATE ${JMENGINE_SOURCES} ${JMENGINE_PUBLIC_HEADERS})

# 在 Visual Studio Solution Explorer 中按真实目录显示 Header Files / Source Files。
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${JMENGINE_PUBLIC_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${JMENGINE_SOURCES})
set_target_properties(JMEngine PROPERTIES FOLDER "JMEngine")

target_include_directories(JMEngine
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# 强制整个库使用 C++17。
# CXX_STANDARD_REQUIRED=YES 可以避免某些 VS/CMake 组合退回到较老语言标准。
if(_JMENGINE_POISSON_SRC_DIR)
    target_include_directories(JMEngine PRIVATE "${_JMENGINE_POISSON_PATCHED_DIR}")
    target_compile_definitions(JMEngine PRIVATE JMENGINE_HAS_POISSONRECON=1)
    if(JMENGINE_POISSON_BIG_DATA)
        target_compile_definitions(JMEngine PRIVATE BIG_DATA=1)
    endif()

    # Build the upstream SurfaceTrimmer translation unit unchanged. The editor calls this
    # official executable instead of carrying a second, reimplemented trimming algorithm.
    if(EXISTS "${_JMENGINE_POISSON_PATCHED_DIR}/SurfaceTrimmer.cpp")
        add_executable(JMEngine_surface_trimmer_official
            "${_JMENGINE_POISSON_PATCHED_DIR}/SurfaceTrimmer.cpp")
        target_include_directories(JMEngine_surface_trimmer_official PRIVATE
            "${_JMENGINE_POISSON_PATCHED_DIR}")
        target_compile_features(JMEngine_surface_trimmer_official PRIVATE cxx_std_17)
        set_target_properties(JMEngine_surface_trimmer_official PROPERTIES
            CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
            FOLDER "ThirdParty/PoissonRecon")
        if(JMENGINE_POISSON_BIG_DATA)
            target_compile_definitions(JMEngine_surface_trimmer_official PRIVATE BIG_DATA=1)
        endif()
        if(OpenMP_CXX_FOUND)
            target_link_libraries(JMEngine_surface_trimmer_official PRIVATE OpenMP::OpenMP_CXX)
        endif()
        if(MSVC)
            target_compile_definitions(JMEngine_surface_trimmer_official PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS)
        endif()
        add_dependencies(JMEngine JMEngine_surface_trimmer_official)
        target_compile_definitions(JMEngine PRIVATE
            JMENGINE_OFFICIAL_SURFACE_TRIMMER_EXE="$<TARGET_FILE:JMEngine_surface_trimmer_official>")
    else()
        message(FATAL_ERROR "PoissonRecon source is missing Src/SurfaceTrimmer.cpp")
    endif()
endif()

target_compile_features(JMEngine PUBLIC cxx_std_17)
set_target_properties(JMEngine PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    # Windows.h 默认会定义 min/max 宏，会破坏 std::min/std::max。
    # 设为 PUBLIC，使所有链接 JMEngine 的 Windows 示例/宿主程序都继承该定义。
    target_compile_definitions(JMEngine PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)

    # /W4、/permissive- 只约束库自身源码。
    target_compile_options(JMEngine PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/W4>
        $<$<COMPILE_LANGUAGE:CXX>:/permissive->)

    # 这里必须是 PUBLIC，而不能是 PRIVATE。
    # 原因：公开头文件中包含大量 UTF-8 中文注释，basic.cpp、Qt 示例以及用户自己的程序
    # 在编译这些头文件时也必须使用 UTF-8。否则某些 Windows 系统代码页会把 UTF-8
    # 多字节字符错误解释，进而产生 C2143/C2447/C4430 等“看似 C++ 语法错误”的连锁报错。
    # /Zc:__cplusplus 同时让 MSVC 的 __cplusplus 正确反映 /std:c++17。
    target_compile_options(JMEngine PUBLIC
        $<$<COMPILE_LANGUAGE:CXX>:/utf-8>
        $<$<COMPILE_LANGUAGE:CXX>:/Zc:__cplusplus>)
else()
    target_compile_options(JMEngine PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(JMENGINE_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
    target_link_libraries(JMEngine PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(JMEngine PUBLIC JMENGINE_USE_OPENMP=1)
    message(STATUS "JMEngine: OpenMP acceleration enabled")
else()
    message(STATUS "JMEngine: OpenMP not found/disabled; using serial CPU path")
endif()

# -----------------------------
# CPU / CUDA texture mapping library (no Qt/OpenCV/VTK dependency)
# -----------------------------
if(JMENGINE_ENABLE_TEXTURE_MAPPING)
    add_library(JMEngine_texture STATIC
        include/JMEngine/texture/TextureMapper.h
        src/texture/TextureMapper.cpp
        src/texture/TextureMapperCuda.cpp
        src/texture/TextureMapperCuda.h
        src/texture/TextureMapperCudaApi.h
    )
    target_include_directories(JMEngine_texture
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(JMEngine_texture PUBLIC JMEngine)
    target_compile_features(JMEngine_texture PUBLIC cxx_std_17)
    set_target_properties(JMEngine_texture PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        FOLDER "JMEngine"
    )
    if(MSVC)
        target_compile_options(JMEngine_texture PUBLIC
            $<$<COMPILE_LANGUAGE:CXX>:/utf-8>
            $<$<COMPILE_LANGUAGE:CXX>:/Zc:__cplusplus>)
    else()
        target_compile_options(JMEngine_texture PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    if(JMENGINE_ENABLE_TEXTURE_CUDA)
        include(CheckLanguage)
        check_language(CUDA)
        if(CMAKE_CUDA_COMPILER)
            enable_language(CUDA)
            find_package(CUDAToolkit QUIET)
            if(CUDAToolkit_FOUND)
                target_sources(JMEngine_texture PRIVATE src/texture/TextureMapperCuda.cu)
                target_link_libraries(JMEngine_texture PRIVATE CUDA::cudart)
                target_compile_definitions(JMEngine_texture PRIVATE JMENGINE_TEXTURE_HAS_CUDA=1)
                set_target_properties(JMEngine_texture PROPERTIES
                    CUDA_STANDARD 17
                    CUDA_STANDARD_REQUIRED YES
                    CUDA_SEPARABLE_COMPILATION OFF
                )
                message(STATUS "JMEngine: texture mapping CUDA backend enabled")
            else()
                message(STATUS "JMEngine: CUDA toolkit not found; texture mapping uses CPU backend")
            endif()
        else()
            message(STATUS "JMEngine: CUDA compiler not found; texture mapping uses CPU backend")
        endif()
    endif()
endif()

# -----------------------------
# 基础命令行示例
# -----------------------------
if(JMENGINE_BUILD_EXAMPLES)
    add_executable(JMEngine_example examples/basic.cpp)
    target_link_libraries(JMEngine_example PRIVATE JMEngine)
    set_target_properties(JMEngine_example PROPERTIES FOLDER "Examples")
endif()

# -----------------------------
# Core 单元/集成测试
# -----------------------------
if(JMENGINE_BUILD_TESTS) 
    enable_testing()
    add_executable(JMEngine_core_tests tests/core_tests.cpp)
    target_link_libraries(JMEngine_core_tests PRIVATE JMEngine)
    target_compile_definitions(JMEngine_core_tests PRIVATE
        JMENGINE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
    set_target_properties(JMEngine_core_tests PROPERTIES FOLDER "Tests")
    add_test(NAME JMEngine_core_tests COMMAND JMEngine_core_tests)
    if(JMENGINE_ENABLE_TEXTURE_MAPPING)
        add_executable(JMEngine_texture_tests tests/texture_tests.cpp)
        target_link_libraries(JMEngine_texture_tests PRIVATE JMEngine_texture)
        set_target_properties(JMEngine_texture_tests PROPERTIES FOLDER "Tests")
        add_test(NAME JMEngine_texture_tests COMMAND JMEngine_texture_tests)

        if(JMENGINE_TEXTURE_DATASET_TEST)

    find_package(Python3 COMPONENTS Interpreter REQUIRED)

    set(JMENGINE_TEXTURE_BOP_CASE_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/test_data/tiny_texture")

    set(JMENGINE_TEXTURE_BOP_CACHE_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/test_data/_cache")

        # ============================================
        # VS 中可见的 prepare_data target
        # ============================================
        add_custom_target(
            JMEngine_texture_bop_prepare_data

            COMMAND
                "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tools/prepare_tiny_texture_case.py"
                --out "${JMENGINE_TEXTURE_BOP_CASE_DIR}"
                --cache "${JMENGINE_TEXTURE_BOP_CACHE_DIR}"
                --scene-id 1
                --object-id 1
                --camera-count 6
                --max-image-width 256

            WORKING_DIRECTORY
                "${CMAKE_CURRENT_SOURCE_DIR}"

            COMMENT
                "Preparing tiny local texture dataset..."

            VERBATIM
        )

        set_target_properties(
            JMEngine_texture_bop_prepare_data
            PROPERTIES
            FOLDER "Tests"
        )

        # ============================================
        # CTest prepare
        # ============================================
        add_test(
            NAME JMEngine_texture_bop_prepare
            COMMAND
                "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tools/prepare_tiny_texture_case.py"
                --out "${JMENGINE_TEXTURE_BOP_CASE_DIR}"
                --cache "${JMENGINE_TEXTURE_BOP_CACHE_DIR}"
                --scene-id 1
                --object-id 1
                --camera-count 6
                --max-image-width 256
        )

        set_tests_properties(
            JMEngine_texture_bop_prepare
            PROPERTIES
            FIXTURES_SETUP JMEngine_texture_bop_data
            TIMEOUT 1200
            LABELS "texture;dataset"
        )

        # ============================================
        # Dataset Test
        # ============================================
        add_executable(
            JMEngine_texture_bop_dataset_test
            tests/texture_bop_dataset_test.cpp
        )

        target_link_libraries(
            JMEngine_texture_bop_dataset_test
            PRIVATE
            JMEngine_texture
        )

        set_target_properties(
            JMEngine_texture_bop_dataset_test
            PROPERTIES
            FOLDER "Tests"
        )

        add_test(
            NAME JMEngine_texture_bop_dataset_test
            COMMAND
                JMEngine_texture_bop_dataset_test
                "${JMENGINE_TEXTURE_BOP_CASE_DIR}"
                "${CMAKE_CURRENT_BINARY_DIR}/test_output/tiny_texture"
        )

        set_tests_properties(
            JMEngine_texture_bop_dataset_test
            PROPERTIES
            FIXTURES_REQUIRED JMEngine_texture_bop_data
            TIMEOUT 1200
            LABELS "texture;dataset"
        )

        endif()

    endif()
endif()

# 三套图形测试共用的辅助头文件。
set(JMENGINE_EXAMPLE_COMMON examples/common/ExampleUtils.h)

# -----------------------------
# 唯一 Qt 编辑器：同一套 UI / Interaction / ModelManager，只抽象渲染后端
# -----------------------------
# 自动选择规则：
#   Windows / x86_64 Desktop -> Desktop OpenGL 2.1 + VAO + RGB24 Picking
#   ARM64/aarch64 Linux      -> OpenGL ES 3.1 + VAO + RGB24 Picking
# 如自动判断不符合实际环境，可通过 -DJMENGINE_RENDER_BACKEND=... 手工覆盖。
set(_JMENGINE_SELECTED_BACKEND "${JMENGINE_RENDER_BACKEND}")
if(_JMENGINE_SELECTED_BACKEND STREQUAL "AUTO")
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _JMENGINE_CPU)
    if(ANDROID)
        set(_JMENGINE_SELECTED_BACKEND "GLES31")
    elseif(UNIX AND NOT APPLE AND (_JMENGINE_CPU MATCHES "aarch64" OR _JMENGINE_CPU MATCHES "arm64"))
        set(_JMENGINE_SELECTED_BACKEND "GLES31")
    else()
        set(_JMENGINE_SELECTED_BACKEND "DESKTOP_GL21")
    endif()
endif()

message(STATUS "JMEngine Qt renderer backend: ${_JMENGINE_SELECTED_BACKEND} (CPU=${CMAKE_SYSTEM_PROCESSOR})")

if(JMENGINE_BUILD_QT_EDITOR)
    find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets QUIET)
    if(Qt6_FOUND)
        # RenderBackend 属于库层，而不是 Qt 示例业务代码。
        # 单独做成 JMEngine_render，避免无 Qt/OpenGL 的 JMEngine Core 被 Qt 依赖污染。
        set(JMENGINE_RENDER_PUBLIC_HEADERS
            include/JMEngine/render/RenderBackend.h
        )
        set(JMENGINE_RENDER_SOURCES
            src/render/RenderBackend.cpp
        )
        add_library(JMEngine_render STATIC)
        target_sources(JMEngine_render PRIVATE
            ${JMENGINE_RENDER_SOURCES}
            ${JMENGINE_RENDER_PUBLIC_HEADERS}
        )
        target_include_directories(JMEngine_render
            PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
        )
        target_link_libraries(JMEngine_render PUBLIC JMEngine Qt6::OpenGL)
        target_compile_features(JMEngine_render PUBLIC cxx_std_17)
        set_target_properties(JMEngine_render PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED YES
            CXX_EXTENSIONS NO
            FOLDER "JMEngine"
        )
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${JMENGINE_RENDER_PUBLIC_HEADERS})
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${JMENGINE_RENDER_SOURCES})

        set(JMENGINE_QT_EDITOR_SOURCES
            examples/qt_editor/main.cpp
            examples/qt_editor/MainWindow.cpp
            examples/qt_editor/MainWindow.h
            examples/qt_editor/PointCloudWidget.cpp
            examples/qt_editor/PointCloudWidget.h
            examples/qt_editor/ProcessingDialog.cpp
            examples/qt_editor/ProcessingDialog.h
            ${JMENGINE_EXAMPLE_COMMON}
        )
        if(ANDROID)
            # qt_add_executable creates the Android deployment metadata consumed by androiddeployqt.
            qt_add_executable(JMEngine_qt_editor MANUAL_FINALIZATION ${JMENGINE_QT_EDITOR_SOURCES})
            set_target_properties(JMEngine_qt_editor PROPERTIES
                QT_ANDROID_PACKAGE_NAME "com.jmengine.viewer"
                QT_ANDROID_APP_NAME "JMEngine"
                QT_ANDROID_VERSION_CODE 1200
                QT_ANDROID_VERSION_NAME "12.0"
            )
            # Keep camera_models.json available inside the APK. The editor copies it to a
            # writable AppDataLocation on first Android launch.
            qt_add_resources(JMEngine_qt_editor JMEngine_android_resources
                PREFIX "/"
                BASE "${CMAKE_CURRENT_SOURCE_DIR}/examples/qt_editor"
                FILES examples/qt_editor/config/camera_models.json)
        else()
            add_executable(JMEngine_qt_editor ${JMENGINE_QT_EDITOR_SOURCES})
        endif()
        target_link_libraries(JMEngine_qt_editor PRIVATE
            JMEngine JMEngine_render Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)
        if(JMENGINE_ENABLE_TEXTURE_MAPPING)
            target_link_libraries(JMEngine_qt_editor PRIVATE JMEngine_texture)
            target_compile_definitions(JMEngine_qt_editor PRIVATE JMENGINE_HAS_TEXTURE_MAPPING=1)
        endif()
        target_compile_features(JMEngine_qt_editor PRIVATE cxx_std_17)

        # Application-layer debugging policy:
        # RelWithDebInfo keeps debug symbols, but disable optimization for the
        # Qt editor/application target only. JMEngine algorithm libraries keep
        # their normal RelWithDebInfo optimization settings.
        if(MSVC)
            target_compile_options(JMEngine_qt_editor PRIVATE
                $<$<CONFIG:RelWithDebInfo>:/Od>
                $<$<CONFIG:RelWithDebInfo>:/Ob0>)
        else()
            target_compile_options(JMEngine_qt_editor PRIVATE
                $<$<CONFIG:RelWithDebInfo>:-O0>)
        endif()

        # -------------------------------------------------------------
        # Windows bundled third_party integration
        #
        # Fixed local layout:
        #   <project>/third_party/win32/eigen3
        #   <project>/third_party/win32/opencv
        #   <project>/third_party/win32/rulermvs
        #
        # OpenCV is linked manually instead of find_package(OpenCV), because
        # the bundled OpenCV 4.8.0 package may have Visual Studio/export config
        # compatibility problems on the current development machine.
        # Keep the Windows scanner SDK layout configurable.  Android/Linux builds
        # never enter this block.  On Windows we first honour an explicit cache
        # path, then probe both the historical and the cross-platform layouts.
        set(JMENGINE_WIN32_THIRD_PARTY_ROOT "" CACHE PATH
            "Windows third-party root containing eigen3, opencv and rulermvs")

        set(_JMENGINE_WIN32_THIRD_CANDIDATES)
        if(JMENGINE_WIN32_THIRD_PARTY_ROOT)
            list(APPEND _JMENGINE_WIN32_THIRD_CANDIDATES
                "${JMENGINE_WIN32_THIRD_PARTY_ROOT}")
        endif()
        list(APPEND _JMENGINE_WIN32_THIRD_CANDIDATES
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party/win32"
            "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/win32"
            "${PROJECT_SOURCE_DIR}/third_party/win32"
            "${PROJECT_SOURCE_DIR}/../third_party/win32"
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
            "${PROJECT_SOURCE_DIR}/third_party")

        set(THIRD_PATH "")
        if(WIN32)
            foreach(_candidate IN LISTS _JMENGINE_WIN32_THIRD_CANDIDATES)
                get_filename_component(_candidate_norm "${_candidate}" ABSOLUTE)
                if(EXISTS "${_candidate_norm}/rulermvs"
                   AND EXISTS "${_candidate_norm}/opencv"
                   AND EXISTS "${_candidate_norm}/eigen3")
                    set(THIRD_PATH "${_candidate_norm}")
                    break()
                endif()
            endforeach()
        endif()

        if(THIRD_PATH)
            set(JMENGINE_EIGEN3_ROOT "${THIRD_PATH}/eigen3")
            set(JMENGINE_OPENCV_ROOT "${THIRD_PATH}/opencv")
            set(JMENGINE_RULERMVS_ROOT "${THIRD_PATH}/rulermvs")
        else()
            # Keep deterministic paths for diagnostics/cache editing even when
            # auto-detection did not find the SDK.
            set(JMENGINE_EIGEN3_ROOT "")
            set(JMENGINE_OPENCV_ROOT "")
            set(JMENGINE_RULERMVS_ROOT "")
        endif()

        set(_JMENGINE_RULERMVS_ENABLED FALSE)
        if(WIN32 AND THIRD_PATH)
            find_path(JMENGINE_RULERMVS_INCLUDE_DIR
                NAMES rulermvs.hpp
                PATHS
                    "${JMENGINE_RULERMVS_ROOT}/include"
                    "${JMENGINE_RULERMVS_ROOT}"
                NO_DEFAULT_PATH)
            find_path(JMENGINE_DBOW3_INCLUDE_DIR
                NAMES DBoW3/DBoW3.h
                PATHS
                    "${JMENGINE_RULERMVS_ROOT}/include"
                    "${JMENGINE_RULERMVS_ROOT}/include/DBoW3"
                    "${JMENGINE_RULERMVS_ROOT}"
                NO_DEFAULT_PATH)
            find_path(JMENGINE_RULERMVS_LIB_DIR
                NAMES rulermvs.lib rulerMVS.lib
                PATHS
                    "${JMENGINE_RULERMVS_ROOT}/lib"
                    "${JMENGINE_RULERMVS_ROOT}/lib/x64"
                    "${JMENGINE_RULERMVS_ROOT}/x64/lib"
                NO_DEFAULT_PATH)

            if(NOT JMENGINE_RULERMVS_INCLUDE_DIR OR NOT JMENGINE_DBOW3_INCLUDE_DIR)
                message(WARNING
                    "rulermvs headers are incomplete under ${JMENGINE_RULERMVS_ROOT}; "
                    "rulermvs.hpp=${JMENGINE_RULERMVS_INCLUDE_DIR}, DBoW3=${JMENGINE_DBOW3_INCLUDE_DIR}")
            elseif(NOT JMENGINE_RULERMVS_LIB_DIR)
                # Some SDK packages use non-standard import library names.  If
                # a lib directory exists, accept it and glob the import libs below.
                foreach(_lib_candidate
                        "${JMENGINE_RULERMVS_ROOT}/lib"
                        "${JMENGINE_RULERMVS_ROOT}/lib/x64"
                        "${JMENGINE_RULERMVS_ROOT}/x64/lib")
                    if(EXISTS "${_lib_candidate}")
                        set(JMENGINE_RULERMVS_LIB_DIR "${_lib_candidate}")
                        break()
                    endif()
                endforeach()
            endif()
        endif()

        if(WIN32
           AND THIRD_PATH
           AND JMENGINE_RULERMVS_INCLUDE_DIR
           AND JMENGINE_DBOW3_INCLUDE_DIR
           AND JMENGINE_RULERMVS_LIB_DIR)

            # Eigen3 is header-only. Support common directory layouts.
            find_path(JMENGINE_EIGEN3_INCLUDE_DIR
                NAMES Eigen/Core
                PATHS
                    "${JMENGINE_EIGEN3_ROOT}"
                    "${JMENGINE_EIGEN3_ROOT}/include"
                    "${JMENGINE_EIGEN3_ROOT}/include/eigen3"
                NO_DEFAULT_PATH)

            if(NOT JMENGINE_EIGEN3_INCLUDE_DIR)
                message(FATAL_ERROR
                    "Eigen3 was not found under ${JMENGINE_EIGEN3_ROOT}. "
                    "Expected Eigen/Core in eigen3, eigen3/include, or eigen3/include/eigen3.")
            endif()

            # ---------------------------------------------------------
            # OpenCV 4.8.0 - manual include/lib configuration
            # ---------------------------------------------------------
            find_path(JMENGINE_OPENCV_INCLUDE_DIR
                NAMES opencv2/core.hpp
                PATHS
                    "${JMENGINE_OPENCV_ROOT}/include"
                    "${JMENGINE_OPENCV_ROOT}/build/include"
                NO_DEFAULT_PATH)
            if(NOT JMENGINE_OPENCV_INCLUDE_DIR)
                message(FATAL_ERROR
                    "OpenCV headers were not found under ${JMENGINE_OPENCV_ROOT}")
            endif()
            set(OpenCV_INCLUDE_DIRS "${JMENGINE_OPENCV_INCLUDE_DIR}")

            # Locate the prebuilt OpenCV import-library directory.  Support the
            # old opencv/lib layout and official opencv/x64/vc*/lib packages.
            set(JMENGINE_OPENCV_LIB_DIR "")
            foreach(_opencv_lib_candidate
                    "${JMENGINE_OPENCV_ROOT}/lib")
                if(EXISTS "${_opencv_lib_candidate}/opencv_core480.lib")
                    set(JMENGINE_OPENCV_LIB_DIR "${_opencv_lib_candidate}")
                    break()
                endif()
            endforeach()
            if(NOT JMENGINE_OPENCV_LIB_DIR)
                file(GLOB _opencv_vc_lib_dirs LIST_DIRECTORIES TRUE
                    "${JMENGINE_OPENCV_ROOT}/x64/vc*/lib")
                foreach(_opencv_lib_candidate IN LISTS _opencv_vc_lib_dirs)
                    if(EXISTS "${_opencv_lib_candidate}/opencv_core480.lib")
                        set(JMENGINE_OPENCV_LIB_DIR "${_opencv_lib_candidate}")
                        break()
                    endif()
                endforeach()
            endif()
            if(NOT JMENGINE_OPENCV_LIB_DIR)
                message(FATAL_ERROR
                    "OpenCV 4.8 import libraries were not found under ${JMENGINE_OPENCV_ROOT}")
            endif()

            set(OpenCV_LIBS
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_core480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_imgcodecs480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_highgui480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_imgproc480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudaarithm480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudabgsegm480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudacodec480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudafeatures2d480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudafilters480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudaimgproc480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudalegacy480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudaobjdetect480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudaoptflow480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudastereo480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudawarping480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_cudev480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_dnn_objdetect480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_dnn_superres480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_dnn480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_video480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_ccalib480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_videoio480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_flann480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_features2d480.lib
                ${JMENGINE_OPENCV_LIB_DIR}/opencv_calib3d480.lib
            )

            # Fail during CMake configure instead of waiting for VS linker errors.
            foreach(_opencv_lib IN LISTS OpenCV_LIBS)
                if(NOT EXISTS "${_opencv_lib}")
                    message(FATAL_ERROR "Missing OpenCV library: ${_opencv_lib}")
                endif()
            endforeach()

            message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
            message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

            # rulermvs SDK libraries. Link all import libraries shipped in lib/.
            file(GLOB _JMENGINE_RULERMVS_LIBS CONFIGURE_DEPENDS
                "${JMENGINE_RULERMVS_LIB_DIR}/*.lib")
            if(NOT _JMENGINE_RULERMVS_LIBS)
                message(FATAL_ERROR
                    "No rulermvs .lib files were found in ${JMENGINE_RULERMVS_LIB_DIR}")
            endif()

            target_include_directories(JMEngine PUBLIC
                "${JMENGINE_RULERMVS_INCLUDE_DIR}"
                "${JMENGINE_DBOW3_INCLUDE_DIR}"
                "${JMENGINE_EIGEN3_INCLUDE_DIR}"
                ${OpenCV_INCLUDE_DIRS}
            )

            target_link_libraries(JMEngine PUBLIC
                ${_JMENGINE_RULERMVS_LIBS}
                ${OpenCV_LIBS}
                strmiids ole32 oleaut32
            )

            target_compile_definitions(JMEngine PUBLIC JMENGINE_HAS_RULERMVS=1)
            set(_JMENGINE_RULERMVS_ENABLED TRUE)

            message(STATUS "JMEngine rulermvs scan integration: enabled")
            message(STATUS "  THIRD_PATH: ${THIRD_PATH}")
            message(STATUS "  rulermvs:   ${JMENGINE_RULERMVS_ROOT}")
            message(STATUS "  OpenCV:     ${JMENGINE_OPENCV_ROOT}")
            message(STATUS "  Eigen3:     ${JMENGINE_EIGEN3_INCLUDE_DIR}")

            # Runtime deployment: copy rulermvs DLLs beside JMEngine_qt_editor.exe.
            file(GLOB _JMENGINE_RULERMVS_DLLS CONFIGURE_DEPENDS
                "${JMENGINE_RULERMVS_ROOT}/bin/*.dll")
            if(_JMENGINE_RULERMVS_DLLS)
                add_custom_command(TARGET JMEngine_qt_editor POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            ${_JMENGINE_RULERMVS_DLLS}
                            "$<TARGET_FILE_DIR:JMEngine_qt_editor>"
                    COMMENT "Copying rulermvs runtime DLLs"
                    COMMAND_EXPAND_LISTS
                    VERBATIM)
            endif()

            # Copy bundled OpenCV runtime DLLs when present.
            # Common layouts supported:
            #   opencv/bin/*.dll
            #   opencv/x64/vc*/bin/*.dll
            file(GLOB_RECURSE _JMENGINE_OPENCV_DLLS CONFIGURE_DEPENDS
                "${JMENGINE_OPENCV_ROOT}/bin/*.dll"
                "${JMENGINE_OPENCV_ROOT}/x64/*/bin/*.dll")
            if(_JMENGINE_OPENCV_DLLS)
                add_custom_command(TARGET JMEngine_qt_editor POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            ${_JMENGINE_OPENCV_DLLS}
                            "$<TARGET_FILE_DIR:JMEngine_qt_editor>"
                    COMMENT "Copying OpenCV runtime DLLs"
                    COMMAND_EXPAND_LISTS
                    VERBATIM)
            endif()

            # Camera VID/PID -> model profile used by the scan UI and capture worker.
            add_custom_command(TARGET JMEngine_qt_editor POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                        "$<TARGET_FILE_DIR:JMEngine_qt_editor>/config"
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${CMAKE_CURRENT_SOURCE_DIR}/examples/qt_editor/config/camera_models.json"
                        "$<TARGET_FILE_DIR:JMEngine_qt_editor>/config/camera_models.json"
                COMMENT "Copying camera model configuration"
                VERBATIM)
        endif()

        if(WIN32 AND NOT _JMENGINE_RULERMVS_ENABLED)
            message(WARNING
                "JMEngine rulermvs scan integration: disabled. "
                "Set -DJMENGINE_WIN32_THIRD_PARTY_ROOT=<path> to the directory "
                "containing eigen3, opencv and rulermvs. Auto-probed: "
                "${_JMENGINE_WIN32_THIRD_CANDIDATES}")
        endif()

        if(_JMENGINE_SELECTED_BACKEND STREQUAL "GLES31")
            target_compile_definitions(JMEngine_render PRIVATE JMENGINE_RENDER_GLES31=1)
            target_compile_definitions(JMEngine_qt_editor PRIVATE JMENGINE_RENDER_GLES31=1)
        elseif(_JMENGINE_SELECTED_BACKEND STREQUAL "DESKTOP_GL21")
            target_compile_definitions(JMEngine_render PRIVATE JMENGINE_RENDER_DESKTOP_GL21=1)
            target_compile_definitions(JMEngine_qt_editor PRIVATE JMENGINE_RENDER_DESKTOP_GL21=1)
        else()
            message(FATAL_ERROR "Unknown JMENGINE_RENDER_BACKEND=${_JMENGINE_SELECTED_BACKEND}")
        endif()

        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/examples" PREFIX "Example Files" FILES
            examples/qt_editor/main.cpp
            examples/qt_editor/MainWindow.cpp
            examples/qt_editor/MainWindow.h
            examples/qt_editor/PointCloudWidget.cpp
            examples/qt_editor/PointCloudWidget.h
            examples/qt_editor/ProcessingDialog.cpp
            examples/qt_editor/ProcessingDialog.h
            examples/qt_editor/config/camera_models.json
            ${JMENGINE_EXAMPLE_COMMON})
        set_target_properties(JMEngine_qt_editor PROPERTIES FOLDER "Examples/Qt")

        # Windows 下自动部署 Qt + MSVC Runtime。
        if(WIN32)
            get_target_property(_JMENGINE_QMAKE_EXECUTABLE Qt6::qmake IMPORTED_LOCATION)
            if(_JMENGINE_QMAKE_EXECUTABLE)
                get_filename_component(_JMENGINE_QT_BIN_DIR "${_JMENGINE_QMAKE_EXECUTABLE}" DIRECTORY)
                find_program(JMENGINE_WINDEPLOYQT_EXECUTABLE
                    NAMES windeployqt windeployqt.exe
                    HINTS "${_JMENGINE_QT_BIN_DIR}"
                    NO_DEFAULT_PATH)
            endif()
            if(NOT JMENGINE_WINDEPLOYQT_EXECUTABLE)
                find_program(JMENGINE_WINDEPLOYQT_EXECUTABLE NAMES windeployqt windeployqt.exe)
            endif()
            if(JMENGINE_WINDEPLOYQT_EXECUTABLE)
                add_custom_command(TARGET JMEngine_qt_editor POST_BUILD
                    COMMAND "${JMENGINE_WINDEPLOYQT_EXECUTABLE}"
                            --compiler-runtime --no-translations
                            --dir "$<TARGET_FILE_DIR:JMEngine_qt_editor>"
                            "$<TARGET_FILE:JMEngine_qt_editor>"
                    COMMENT "Deploying Qt and MSVC runtime dependencies with windeployqt"
                    VERBATIM)
            else()
                message(WARNING "windeployqt not found; Qt runtime files will not be copied automatically")
            endif()
        endif()

        if(ANDROID)
            qt_finalize_executable(JMEngine_qt_editor)
            message(STATUS "JMEngine Android APK target configured (GLES 3.1, package com.jmengine.viewer)")
        endif()
    else()
        message(STATUS "Qt6 Widgets/OpenGL/OpenGLWidgets not found; skipping unified Qt editor")
    endif()
endif()

# -----------------------------
# Windows 原生 Win32/WGL OpenGL 测试
# -----------------------------
# 不依赖 GLEW/GLAD，直接链接 Windows 系统 opengl32。
# Windows 示例固定使用 OpenGL 2.1 兼容 RGB24 Color Picking。
# 不依赖 R32UI/GLSL330/FBO：直接在后缓冲执行隐藏 ID Pass 后 glReadPixels，
# 因此即便仅有传统 WGL 兼容上下文也可验证 GPU 选择。
if(WIN32 AND JMENGINE_BUILD_WINDOWS_OPENGL_EXAMPLE)
    add_executable(JMEngine_windows_opengl
        examples/windows_opengl/main.cpp
        ${JMENGINE_EXAMPLE_COMMON}
    )
    target_link_libraries(JMEngine_windows_opengl PRIVATE JMEngine opengl32)
    target_compile_definitions(JMEngine_windows_opengl PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/examples" PREFIX "Example Files" FILES
        examples/windows_opengl/main.cpp ${JMENGINE_EXAMPLE_COMMON})
    set_target_properties(JMEngine_windows_opengl PROPERTIES FOLDER "Examples/Windows")
endif()

# -----------------------------
# RK3588 / Linux EGL + GLES3 + R32UI GPU Picking
# -----------------------------
# 只有检测到 EGL/GLES3 开发文件后才生成目标，避免普通 PC 环境配置失败。
if(UNIX AND NOT APPLE AND JMENGINE_BUILD_RK3588_GLES_EXAMPLE)
    find_path(JMENGINE_EGL_INCLUDE_DIR EGL/egl.h)
    find_path(JMENGINE_GLES3_INCLUDE_DIR GLES3/gl3.h)
    find_library(JMENGINE_EGL_LIBRARY EGL)
    find_library(JMENGINE_GLES3_LIBRARY GLESv2)
    if(JMENGINE_EGL_INCLUDE_DIR AND JMENGINE_GLES3_INCLUDE_DIR AND JMENGINE_EGL_LIBRARY AND JMENGINE_GLES3_LIBRARY)
        add_executable(JMEngine_rk3588_gles
            examples/rk3588_gles/main.cpp
            ${JMENGINE_EXAMPLE_COMMON}
        )
        target_include_directories(JMEngine_rk3588_gles PRIVATE
            ${JMENGINE_EGL_INCLUDE_DIR} ${JMENGINE_GLES3_INCLUDE_DIR})
        target_link_libraries(JMEngine_rk3588_gles PRIVATE
            JMEngine ${JMENGINE_EGL_LIBRARY} ${JMENGINE_GLES3_LIBRARY})
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/examples" PREFIX "Example Files" FILES
            examples/rk3588_gles/main.cpp ${JMENGINE_EXAMPLE_COMMON})
        set_target_properties(JMEngine_rk3588_gles PROPERTIES FOLDER "Examples/RK3588")
    else()
        message(STATUS "EGL/GLES3 development files not found; skipping RK3588 GLES example")
    endif()
endif()

# -----------------------------
# 安装规则
# -----------------------------
install(TARGETS JMEngine
    EXPORT JMEngineTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
if(TARGET JMEngine_render)
    install(TARGETS JMEngine_render
        EXPORT JMEngineTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()
install(DIRECTORY include/ DESTINATION include)

# Keep the official SurfaceTrimmer beside runtime editors/tools as a deployment fallback.
if(TARGET JMEngine_qt_editor AND TARGET JMEngine_surface_trimmer_official)
    add_custom_command(TARGET JMEngine_qt_editor POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "$<TARGET_FILE:JMEngine_surface_trimmer_official>"
                "$<TARGET_FILE_DIR:JMEngine_qt_editor>/$<TARGET_FILE_NAME:JMEngine_surface_trimmer_official>"
        COMMENT "Copying official PoissonRecon SurfaceTrimmer beside JMEngine_qt_editor"
        VERBATIM)
endif()
if(TARGET JMEngine_surface_trimmer_official)
    install(TARGETS JMEngine_surface_trimmer_official RUNTIME DESTINATION bin)
endif()
