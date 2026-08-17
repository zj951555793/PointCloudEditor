cmake_minimum_required(VERSION 3.16)
project(PointCloudEditor VERSION 2.3.6 LANGUAGES CXX)

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

option(PCEDITOR_BUILD_EXAMPLES "Build basic example" ON)
option(PCEDITOR_BUILD_TESTS "Build core tests" ON)
option(PCEDITOR_TEXTURE_DATASET_TEST "Enable auto-download BOP LINEMOD Mesh->Texture integration test" ON)
option(PCEDITOR_BUILD_QT_EDITOR "Build the single unified Qt editor" ON)
set(PCEDITOR_RENDER_BACKEND "AUTO" CACHE STRING "Qt renderer backend: AUTO, DESKTOP_GL21, GLES31")
set_property(CACHE PCEDITOR_RENDER_BACKEND PROPERTY STRINGS AUTO DESKTOP_GL21 GLES31)
option(PCEDITOR_BUILD_WINDOWS_OPENGL_EXAMPLE "Build native Win32/WGL backend test" OFF)
option(PCEDITOR_BUILD_RK3588_GLES_EXAMPLE "Build native EGL/GLES backend test" OFF)
option(PCEDITOR_ENABLE_OPENMP "Enable OpenMP acceleration when available" ON)
option(PCEDITOR_ENABLE_TEXTURE_MAPPING "Build CPU/CUDA texture mapping library" ON)
option(PCEDITOR_ENABLE_TEXTURE_CUDA "Enable optional CUDA backend for texture mapping when a CUDA compiler is available" ON)
option(PCEDITOR_ENABLE_POISSONRECON "Enable industrial Kazhdan Poisson reconstruction" ON)
option(PCEDITOR_FETCH_POISSONRECON "Developer fallback: fetch PoissonRecon 18.76 when no bundled/local source exists" OFF)
option(PCEDITOR_REQUIRE_POISSONRECON "Fail configure when the industrial Poisson backend is unavailable" OFF)
option(PCEDITOR_POISSON_BIG_DATA "Enable PoissonRecon BIG_DATA indexing" OFF)
set(PCEDITOR_POISSONRECON_ROOT "" CACHE PATH "Optional external PoissonRecon source tree (directory containing Src/Reconstructors.h)")
set(PCEDITOR_POISSONRECON_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/PoissonRecon" CACHE PATH "Bundled/vendor PoissonRecon 18.76 source tree")

# -----------------------------
# Android / RK3588 GLES platform policy
# -----------------------------
# Android V12 focuses on getting the existing Qt editor running unchanged on arm64-v8a.
# Rendering uses the same GLES 3.1 backend as RK3588 Linux. CUDA/Win32-only integrations
# are disabled automatically. PoissonRecon's current official SurfaceTrimmer integration
# launches a helper executable, which is not an Android-app-safe deployment model, so the
# industrial Poisson backend is disabled on Android until the trimmer is moved in-process.
if(ANDROID)
    set(PCEDITOR_RENDER_BACKEND "GLES31" CACHE STRING "Qt renderer backend" FORCE)
    set(PCEDITOR_ENABLE_TEXTURE_CUDA OFF CACHE BOOL "CUDA is unavailable in the Android NDK build" FORCE)
    set(PCEDITOR_BUILD_WINDOWS_OPENGL_EXAMPLE OFF CACHE BOOL "Win32 example is unavailable on Android" FORCE)
    set(PCEDITOR_BUILD_RK3588_GLES_EXAMPLE OFF CACHE BOOL "Native EGL example is not part of the Android APK" FORCE)
    set(PCEDITOR_BUILD_TESTS OFF CACHE BOOL "Host CTest binaries are not packaged into the Android APK" FORCE)
    set(PCEDITOR_TEXTURE_DATASET_TEST OFF CACHE BOOL "Dataset test is host-side only" FORCE)
    set(PCEDITOR_BUILD_EXAMPLES OFF CACHE BOOL "CLI example is host-side only" FORCE)
    set(PCEDITOR_ENABLE_POISSONRECON OFF CACHE BOOL "Android V12: Poisson SurfaceTrimmer helper is not packaged" FORCE)
    add_compile_definitions(PCEDITOR_PLATFORM_ANDROID=1)
    message(STATUS "PointCloudEditor Android policy: arm64 Qt/Widgets + GLES3.1; CUDA/Poisson helper disabled")
endif()

# OBJ/PLY 大模型的 CPU 后处理可使用 OpenMP 并行。
# OpenMP 为可选依赖：找不到时仍然使用串行实现，保证库/示例可正常构建。
if(PCEDITOR_ENABLE_OPENMP)
    find_package(OpenMP QUIET)
endif()

# 所有公开头文件都显式加入 target_sources。
# 这样使用 Visual Studio CMake Generator 生成 .sln/.vcxproj 时，Header Files 下会显示 .h 文件，
# 不会出现“库能编译，但 VS 项目树里只有 .cpp 没有 .h”的问题。
set(PCEDITOR_PUBLIC_HEADERS
    include/pceditor/Types.h
    include/pceditor/Alignment.h
    include/pceditor/Measurement.h
    include/pceditor/Edit.h
    include/pceditor/PointCloud.h
    include/pceditor/Selection.h
    include/pceditor/PointCloudEditor.h
    include/pceditor/TriangleMesh.h
    include/pceditor/edit/DirtyRange.h
    include/pceditor/edit/EditResult.h
    include/pceditor/edit/PointCloudEditSession.h
    include/pceditor/edit/MeshEditSession.h
    include/pceditor/CpuSelector.h
    include/pceditor/CpuMeshSelector.h
    include/pceditor/MeshSelectionClosure.h
    include/pceditor/IGpuPicker.h
    include/pceditor/PixelIdPicker.h
    include/pceditor/ColorPicking24.h
    include/pceditor/BlockPicking24.h
    include/pceditor/OpenGLPickingShaders.h
    include/pceditor/PointCloudIO.h
    include/pceditor/ObjMeshLoader.h
    include/pceditor/ObjAppearanceLoader.h
    include/pceditor/ObjModelLoader.h
    include/pceditor/MeshUtils.h
    include/pceditor/ModelIO.h
    include/pceditor/processing/Processing.h
    include/pceditor/processing/Parallel.h
    include/pceditor/processing/Operations.h
    include/pceditor/processing/Diagnostics.h
)

set(PCEDITOR_SOURCES
    src/PointCloud.cpp
    src/Alignment.cpp
    src/Measurement.cpp
    src/PointCloudEditor.cpp
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
#   2) Allow an explicit external PCEDITOR_POISSONRECON_ROOT override.
#   3) Network FetchContent is developer-only and OFF by default.
# This keeps cmake-gui / CI / factory machines reproducible and offline by default.
set(_PCEDITOR_POISSON_SRC_DIR "")
set(_PCEDITOR_POISSON_SOURCE_ROOT "")
if(PCEDITOR_ENABLE_POISSONRECON)
    if(PCEDITOR_POISSONRECON_ROOT)
        set(_PCEDITOR_POISSON_SOURCE_ROOT "${PCEDITOR_POISSONRECON_ROOT}")
    elseif(EXISTS "${PCEDITOR_POISSONRECON_VENDOR_DIR}/Src/Reconstructors.h")
        set(_PCEDITOR_POISSON_SOURCE_ROOT "${PCEDITOR_POISSONRECON_VENDOR_DIR}")
    elseif(PCEDITOR_FETCH_POISSONRECON)
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
        file(GLOB_RECURSE _PCEDITOR_POISSON_RECONSTRUCTORS
            "${poissonrecon1876_SOURCE_DIR}/Reconstructors.h"
            "${poissonrecon1876_SOURCE_DIR}/*/Reconstructors.h")
        list(LENGTH _PCEDITOR_POISSON_RECONSTRUCTORS _PCEDITOR_POISSON_HEADER_COUNT)
        if(_PCEDITOR_POISSON_HEADER_COUNT GREATER 0)
            list(GET _PCEDITOR_POISSON_RECONSTRUCTORS 0 _PCEDITOR_POISSON_RECONSTRUCTORS_H)
            get_filename_component(_PCEDITOR_POISSON_SRC_DIR "${_PCEDITOR_POISSON_RECONSTRUCTORS_H}" DIRECTORY)
            get_filename_component(_PCEDITOR_POISSON_SOURCE_ROOT "${_PCEDITOR_POISSON_SRC_DIR}" DIRECTORY)
        else()
            message(FATAL_ERROR "Fetched PoissonRecon 18.76 archive does not contain Reconstructors.h")
        endif()
    endif()

    if(_PCEDITOR_POISSON_SOURCE_ROOT AND NOT _PCEDITOR_POISSON_SRC_DIR)
        if(EXISTS "${_PCEDITOR_POISSON_SOURCE_ROOT}/Src/Reconstructors.h")
            set(_PCEDITOR_POISSON_SRC_DIR "${_PCEDITOR_POISSON_SOURCE_ROOT}/Src")
        elseif(EXISTS "${_PCEDITOR_POISSON_SOURCE_ROOT}/Reconstructors.h")
            set(_PCEDITOR_POISSON_SRC_DIR "${_PCEDITOR_POISSON_SOURCE_ROOT}")
        else()
            message(FATAL_ERROR "PoissonRecon root does not contain Src/Reconstructors.h: ${_PCEDITOR_POISSON_SOURCE_ROOT}")
        endif()
    endif()

    # Guard against accidentally vendoring a different upstream revision.
    if(_PCEDITOR_POISSON_SOURCE_ROOT AND EXISTS "${_PCEDITOR_POISSON_SOURCE_ROOT}/README.md")
        file(READ "${_PCEDITOR_POISSON_SOURCE_ROOT}/README.md" _PCEDITOR_POISSON_README LIMIT 65536)
        string(FIND "${_PCEDITOR_POISSON_README}" "Version 18.76" _PCEDITOR_POISSON_VERSION_POS)
        if(_PCEDITOR_POISSON_VERSION_POS EQUAL -1)
            message(WARNING "PoissonRecon source does not advertise Version 18.76; verify third_party/PoissonRecon before production release")
        endif()
    endif()

    if(NOT _PCEDITOR_POISSON_SRC_DIR AND PCEDITOR_REQUIRE_POISSONRECON)
        message(FATAL_ERROR
            "Industrial PoissonRecon 18.76 is required but not vendored. "
            "Run: python tools/vendor_poissonrecon.py  (or pass --archive <AdaptiveSolvers.zip>)")
    endif()
endif()

# The upstream ThreadPool stores hardware_concurrency() in a private static and uses it explicitly
# in OpenMP num_threads(). To honor PointCloudEditor's CPU-1 policy we copy the header-only source
# into the build tree and add one setter. No solver/FEM code is changed.
if(_PCEDITOR_POISSON_SRC_DIR)
    set(_PCEDITOR_POISSON_PATCHED_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/poissonrecon1876-patched/Src")
    file(REMOVE_RECURSE "${_PCEDITOR_POISSON_PATCHED_DIR}")
    file(MAKE_DIRECTORY "${_PCEDITOR_POISSON_PATCHED_DIR}")
    file(COPY "${_PCEDITOR_POISSON_SRC_DIR}/" DESTINATION "${_PCEDITOR_POISSON_PATCHED_DIR}")
    set(_PCEDITOR_POISSON_MT "${_PCEDITOR_POISSON_PATCHED_DIR}/MultiThreading.h")
    file(READ "${_PCEDITOR_POISSON_MT}" _PCEDITOR_POISSON_MT_TEXT)
    string(FIND "${_PCEDITOR_POISSON_MT_TEXT}" "SetNumThreads" _PCEDITOR_POISSON_HAS_SETTER)
    if(_PCEDITOR_POISSON_HAS_SETTER EQUAL -1)
        string(REPLACE
            "static unsigned int NumThreads( void ){ return _NumThreads; }"
            "static unsigned int NumThreads( void ){ return _NumThreads; }\n\t\tstatic void SetNumThreads( unsigned int n ){ _NumThreads = n ? n : 1; }"
            _PCEDITOR_POISSON_MT_TEXT
            "${_PCEDITOR_POISSON_MT_TEXT}")
        file(WRITE "${_PCEDITOR_POISSON_MT}" "${_PCEDITOR_POISSON_MT_TEXT}")
        string(FIND "${_PCEDITOR_POISSON_MT_TEXT}" "SetNumThreads" _PCEDITOR_POISSON_PATCH_OK)
        if(_PCEDITOR_POISSON_PATCH_OK EQUAL -1)
            message(FATAL_ERROR "PoissonRecon MultiThreading.h layout changed; CPU-1 integration patch could not be applied")
        endif()
    endif()
    message(STATUS "PointCloudEditor: industrial PoissonRecon backend enabled (${_PCEDITOR_POISSON_SRC_DIR})")
else()
    message(STATUS "PointCloudEditor: PoissonRecon 18.76 not vendored; core/editor still build, reconstruction reports backend unavailable")
endif()

add_library(pceditor STATIC)
# 注意：公开头文件只是“安装/包含接口”，不要作为 PUBLIC source 传播给依赖目标。
# 如果写成 PUBLIC，Visual Studio 会把 PointCloudEditor 的 .h 同时显示到 Examples 项目下，
# 看起来像示例工程“公开加入了库头文件”。这里改成 PRIVATE 后：
# - pceditor 项目自身仍能在 Header Files 下看到所有头文件；
# - pceditor_example / Qt / Windows 示例不会重复出现这些头文件；
# - target_include_directories(PUBLIC ...) 仍然保证 #include <pceditor/...> 正常工作。
target_sources(pceditor PRIVATE ${PCEDITOR_SOURCES} ${PCEDITOR_PUBLIC_HEADERS})

# 在 Visual Studio Solution Explorer 中按真实目录显示 Header Files / Source Files。
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${PCEDITOR_PUBLIC_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${PCEDITOR_SOURCES})
set_target_properties(pceditor PROPERTIES FOLDER "PointCloudEditor")

target_include_directories(pceditor
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# 强制整个库使用 C++17。
# CXX_STANDARD_REQUIRED=YES 可以避免某些 VS/CMake 组合退回到较老语言标准。
if(_PCEDITOR_POISSON_SRC_DIR)
    target_include_directories(pceditor PRIVATE "${_PCEDITOR_POISSON_PATCHED_DIR}")
    target_compile_definitions(pceditor PRIVATE PCEDITOR_HAS_POISSONRECON=1)
    if(PCEDITOR_POISSON_BIG_DATA)
        target_compile_definitions(pceditor PRIVATE BIG_DATA=1)
    endif()

    # Build the upstream SurfaceTrimmer translation unit unchanged. The editor calls this
    # official executable instead of carrying a second, reimplemented trimming algorithm.
    if(EXISTS "${_PCEDITOR_POISSON_PATCHED_DIR}/SurfaceTrimmer.cpp")
        add_executable(pceditor_surface_trimmer_official
            "${_PCEDITOR_POISSON_PATCHED_DIR}/SurfaceTrimmer.cpp")
        target_include_directories(pceditor_surface_trimmer_official PRIVATE
            "${_PCEDITOR_POISSON_PATCHED_DIR}")
        target_compile_features(pceditor_surface_trimmer_official PRIVATE cxx_std_17)
        set_target_properties(pceditor_surface_trimmer_official PROPERTIES
            CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
            FOLDER "ThirdParty/PoissonRecon")
        if(PCEDITOR_POISSON_BIG_DATA)
            target_compile_definitions(pceditor_surface_trimmer_official PRIVATE BIG_DATA=1)
        endif()
        if(OpenMP_CXX_FOUND)
            target_link_libraries(pceditor_surface_trimmer_official PRIVATE OpenMP::OpenMP_CXX)
        endif()
        if(MSVC)
            target_compile_definitions(pceditor_surface_trimmer_official PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS)
        endif()
        add_dependencies(pceditor pceditor_surface_trimmer_official)
        target_compile_definitions(pceditor PRIVATE
            PCEDITOR_OFFICIAL_SURFACE_TRIMMER_EXE="$<TARGET_FILE:pceditor_surface_trimmer_official>")
    else()
        message(FATAL_ERROR "PoissonRecon source is missing Src/SurfaceTrimmer.cpp")
    endif()
endif()

target_compile_features(pceditor PUBLIC cxx_std_17)
set_target_properties(pceditor PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    # Windows.h 默认会定义 min/max 宏，会破坏 std::min/std::max。
    # 设为 PUBLIC，使所有链接 pceditor 的 Windows 示例/宿主程序都继承该定义。
    target_compile_definitions(pceditor PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)

    # /W4、/permissive- 只约束库自身源码。
    target_compile_options(pceditor PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/W4>
        $<$<COMPILE_LANGUAGE:CXX>:/permissive->)

    # 这里必须是 PUBLIC，而不能是 PRIVATE。
    # 原因：公开头文件中包含大量 UTF-8 中文注释，basic.cpp、Qt 示例以及用户自己的程序
    # 在编译这些头文件时也必须使用 UTF-8。否则某些 Windows 系统代码页会把 UTF-8
    # 多字节字符错误解释，进而产生 C2143/C2447/C4430 等“看似 C++ 语法错误”的连锁报错。
    # /Zc:__cplusplus 同时让 MSVC 的 __cplusplus 正确反映 /std:c++17。
    target_compile_options(pceditor PUBLIC
        $<$<COMPILE_LANGUAGE:CXX>:/utf-8>
        $<$<COMPILE_LANGUAGE:CXX>:/Zc:__cplusplus>)
else()
    target_compile_options(pceditor PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(PCEDITOR_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
    target_link_libraries(pceditor PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(pceditor PUBLIC PCEDITOR_USE_OPENMP=1)
    message(STATUS "PointCloudEditor: OpenMP acceleration enabled")
else()
    message(STATUS "PointCloudEditor: OpenMP not found/disabled; using serial CPU path")
endif()

# -----------------------------
# CPU / CUDA texture mapping library (no Qt/OpenCV/VTK dependency)
# -----------------------------
if(PCEDITOR_ENABLE_TEXTURE_MAPPING)
    add_library(pceditor_texture STATIC
        include/pceditor/texture/TextureMapper.h
        src/texture/TextureMapper.cpp
        src/texture/TextureMapperCuda.cpp
        src/texture/TextureMapperCuda.h
        src/texture/TextureMapperCudaApi.h
    )
    target_include_directories(pceditor_texture
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(pceditor_texture PUBLIC pceditor)
    target_compile_features(pceditor_texture PUBLIC cxx_std_17)
    set_target_properties(pceditor_texture PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        FOLDER "PointCloudEditor"
    )
    if(MSVC)
        target_compile_options(pceditor_texture PUBLIC
            $<$<COMPILE_LANGUAGE:CXX>:/utf-8>
            $<$<COMPILE_LANGUAGE:CXX>:/Zc:__cplusplus>)
    else()
        target_compile_options(pceditor_texture PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    if(PCEDITOR_ENABLE_TEXTURE_CUDA)
        include(CheckLanguage)
        check_language(CUDA)
        if(CMAKE_CUDA_COMPILER)
            enable_language(CUDA)
            find_package(CUDAToolkit QUIET)
            if(CUDAToolkit_FOUND)
                target_sources(pceditor_texture PRIVATE src/texture/TextureMapperCuda.cu)
                target_link_libraries(pceditor_texture PRIVATE CUDA::cudart)
                target_compile_definitions(pceditor_texture PRIVATE PCEDITOR_TEXTURE_HAS_CUDA=1)
                set_target_properties(pceditor_texture PROPERTIES
                    CUDA_STANDARD 17
                    CUDA_STANDARD_REQUIRED YES
                    CUDA_SEPARABLE_COMPILATION OFF
                )
                message(STATUS "PointCloudEditor: texture mapping CUDA backend enabled")
            else()
                message(STATUS "PointCloudEditor: CUDA toolkit not found; texture mapping uses CPU backend")
            endif()
        else()
            message(STATUS "PointCloudEditor: CUDA compiler not found; texture mapping uses CPU backend")
        endif()
    endif()
endif()

# -----------------------------
# 基础命令行示例
# -----------------------------
if(PCEDITOR_BUILD_EXAMPLES)
    add_executable(pceditor_example examples/basic.cpp)
    target_link_libraries(pceditor_example PRIVATE pceditor)
    set_target_properties(pceditor_example PROPERTIES FOLDER "Examples")
endif()

# -----------------------------
# Core 单元/集成测试
# -----------------------------
if(PCEDITOR_BUILD_TESTS) 
    enable_testing()
    add_executable(pceditor_core_tests tests/core_tests.cpp)
    target_link_libraries(pceditor_core_tests PRIVATE pceditor)
    target_compile_definitions(pceditor_core_tests PRIVATE
        PCEDITOR_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
    set_target_properties(pceditor_core_tests PROPERTIES FOLDER "Tests")
    add_test(NAME pceditor_core_tests COMMAND pceditor_core_tests)
    if(PCEDITOR_ENABLE_TEXTURE_MAPPING)
        add_executable(pceditor_texture_tests tests/texture_tests.cpp)
        target_link_libraries(pceditor_texture_tests PRIVATE pceditor_texture)
        set_target_properties(pceditor_texture_tests PROPERTIES FOLDER "Tests")
        add_test(NAME pceditor_texture_tests COMMAND pceditor_texture_tests)

        if(PCEDITOR_TEXTURE_DATASET_TEST)

    find_package(Python3 COMPONENTS Interpreter REQUIRED)

    set(PCEDITOR_TEXTURE_BOP_CASE_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/test_data/tiny_texture")

    set(PCEDITOR_TEXTURE_BOP_CACHE_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/test_data/_cache")

        # ============================================
        # VS 中可见的 prepare_data target
        # ============================================
        add_custom_target(
            pceditor_texture_bop_prepare_data

            COMMAND
                "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tools/prepare_tiny_texture_case.py"
                --out "${PCEDITOR_TEXTURE_BOP_CASE_DIR}"
                --cache "${PCEDITOR_TEXTURE_BOP_CACHE_DIR}"
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
            pceditor_texture_bop_prepare_data
            PROPERTIES
            FOLDER "Tests"
        )

        # ============================================
        # CTest prepare
        # ============================================
        add_test(
            NAME pceditor_texture_bop_prepare
            COMMAND
                "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tools/prepare_tiny_texture_case.py"
                --out "${PCEDITOR_TEXTURE_BOP_CASE_DIR}"
                --cache "${PCEDITOR_TEXTURE_BOP_CACHE_DIR}"
                --scene-id 1
                --object-id 1
                --camera-count 6
                --max-image-width 256
        )

        set_tests_properties(
            pceditor_texture_bop_prepare
            PROPERTIES
            FIXTURES_SETUP pceditor_texture_bop_data
            TIMEOUT 1200
            LABELS "texture;dataset"
        )

        # ============================================
        # Dataset Test
        # ============================================
        add_executable(
            pceditor_texture_bop_dataset_test
            tests/texture_bop_dataset_test.cpp
        )

        target_link_libraries(
            pceditor_texture_bop_dataset_test
            PRIVATE
            pceditor_texture
        )

        set_target_properties(
            pceditor_texture_bop_dataset_test
            PROPERTIES
            FOLDER "Tests"
        )

        add_test(
            NAME pceditor_texture_bop_dataset_test
            COMMAND
                pceditor_texture_bop_dataset_test
                "${PCEDITOR_TEXTURE_BOP_CASE_DIR}"
                "${CMAKE_CURRENT_BINARY_DIR}/test_output/tiny_texture"
        )

        set_tests_properties(
            pceditor_texture_bop_dataset_test
            PROPERTIES
            FIXTURES_REQUIRED pceditor_texture_bop_data
            TIMEOUT 1200
            LABELS "texture;dataset"
        )

        endif()

    endif()
endif()

# 三套图形测试共用的辅助头文件。
set(PCEDITOR_EXAMPLE_COMMON examples/common/ExampleUtils.h)

# -----------------------------
# 唯一 Qt 编辑器：同一套 UI / Interaction / ModelManager，只抽象渲染后端
# -----------------------------
# 自动选择规则：
#   Windows / x86_64 Desktop -> Desktop OpenGL 2.1 + VAO + RGB24 Picking
#   ARM64/aarch64 Linux      -> OpenGL ES 3.1 + VAO + RGB24 Picking
# 如自动判断不符合实际环境，可通过 -DPCEDITOR_RENDER_BACKEND=... 手工覆盖。
set(_PCEDITOR_SELECTED_BACKEND "${PCEDITOR_RENDER_BACKEND}")
if(_PCEDITOR_SELECTED_BACKEND STREQUAL "AUTO")
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _PCEDITOR_CPU)
    if(ANDROID)
        set(_PCEDITOR_SELECTED_BACKEND "GLES31")
    elseif(UNIX AND NOT APPLE AND (_PCEDITOR_CPU MATCHES "aarch64" OR _PCEDITOR_CPU MATCHES "arm64"))
        set(_PCEDITOR_SELECTED_BACKEND "GLES31")
    else()
        set(_PCEDITOR_SELECTED_BACKEND "DESKTOP_GL21")
    endif()
endif()

message(STATUS "PointCloudEditor Qt renderer backend: ${_PCEDITOR_SELECTED_BACKEND} (CPU=${CMAKE_SYSTEM_PROCESSOR})")

if(PCEDITOR_BUILD_QT_EDITOR)
    find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets QUIET)
    if(Qt6_FOUND)
        # RenderBackend 属于库层，而不是 Qt 示例业务代码。
        # 单独做成 pceditor_render，避免无 Qt/OpenGL 的 pceditor Core 被 Qt 依赖污染。
        set(PCEDITOR_RENDER_PUBLIC_HEADERS
            include/pceditor/render/RenderBackend.h
        )
        set(PCEDITOR_RENDER_SOURCES
            src/render/RenderBackend.cpp
        )
        add_library(pceditor_render STATIC)
        target_sources(pceditor_render PRIVATE
            ${PCEDITOR_RENDER_SOURCES}
            ${PCEDITOR_RENDER_PUBLIC_HEADERS}
        )
        target_include_directories(pceditor_render
            PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
        )
        target_link_libraries(pceditor_render PUBLIC pceditor Qt6::OpenGL)
        target_compile_features(pceditor_render PUBLIC cxx_std_17)
        set_target_properties(pceditor_render PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED YES
            CXX_EXTENSIONS NO
            FOLDER "PointCloudEditor"
        )
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${PCEDITOR_RENDER_PUBLIC_HEADERS})
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${PCEDITOR_RENDER_SOURCES})

        set(PCEDITOR_QT_EDITOR_SOURCES
            examples/qt_editor/main.cpp
            examples/qt_editor/MainWindow.cpp
            examples/qt_editor/MainWindow.h
            examples/qt_editor/PointCloudWidget.cpp
            examples/qt_editor/PointCloudWidget.h
            examples/qt_editor/ProcessingDialog.cpp
            examples/qt_editor/ProcessingDialog.h
            examples/qt_editor/ScanFlowController.cpp
            examples/qt_editor/ScanFlowController.h
            examples/qt_editor/CameraDeviceManager.cpp
            examples/qt_editor/CameraDeviceManager.h
            ${PCEDITOR_EXAMPLE_COMMON}
        )
        if(ANDROID)
            # qt_add_executable creates the Android deployment metadata consumed by androiddeployqt.
            qt_add_executable(pceditor_qt_editor MANUAL_FINALIZATION ${PCEDITOR_QT_EDITOR_SOURCES})
            set_target_properties(pceditor_qt_editor PROPERTIES
                QT_ANDROID_PACKAGE_NAME "com.pceditor.viewer"
                QT_ANDROID_APP_NAME "PointCloudEditor"
                QT_ANDROID_VERSION_CODE 1200
                QT_ANDROID_VERSION_NAME "12.0"
            )
            # Keep camera_models.json available inside the APK. The editor copies it to a
            # writable AppDataLocation on first Android launch.
            qt_add_resources(pceditor_qt_editor pceditor_android_resources
                PREFIX "/"
                BASE "${CMAKE_CURRENT_SOURCE_DIR}/examples/qt_editor"
                FILES examples/qt_editor/config/camera_models.json)
        else()
            add_executable(pceditor_qt_editor ${PCEDITOR_QT_EDITOR_SOURCES})
        endif()
        target_link_libraries(pceditor_qt_editor PRIVATE
            pceditor pceditor_render Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)
        if(WIN32)
            # CameraDeviceManager uses COM + DirectShow device enumeration.
            target_link_libraries(pceditor_qt_editor PRIVATE
                ole32 oleaut32 uuid strmiids)
        endif()
        if(PCEDITOR_ENABLE_TEXTURE_MAPPING)
            target_link_libraries(pceditor_qt_editor PRIVATE pceditor_texture)
            target_compile_definitions(pceditor_qt_editor PRIVATE PCEDITOR_HAS_TEXTURE_MAPPING=1)
        endif()
        target_compile_features(pceditor_qt_editor PRIVATE cxx_std_17)

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
        set(PCEDITOR_WIN32_THIRD_PARTY_ROOT "" CACHE PATH
            "Windows third-party root containing eigen3, opencv and rulermvs")

        set(_PCEDITOR_WIN32_THIRD_CANDIDATES)
        if(PCEDITOR_WIN32_THIRD_PARTY_ROOT)
            list(APPEND _PCEDITOR_WIN32_THIRD_CANDIDATES
                "${PCEDITOR_WIN32_THIRD_PARTY_ROOT}")
        endif()
        list(APPEND _PCEDITOR_WIN32_THIRD_CANDIDATES
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party/win32"
            "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/win32"
            "${PROJECT_SOURCE_DIR}/third_party/win32"
            "${PROJECT_SOURCE_DIR}/../third_party/win32"
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
            "${PROJECT_SOURCE_DIR}/third_party")

        set(THIRD_PATH "")
        if(WIN32)
            foreach(_candidate IN LISTS _PCEDITOR_WIN32_THIRD_CANDIDATES)
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
            set(PCEDITOR_EIGEN3_ROOT "${THIRD_PATH}/eigen3")
            set(PCEDITOR_OPENCV_ROOT "${THIRD_PATH}/opencv")
            set(PCEDITOR_RULERMVS_ROOT "${THIRD_PATH}/rulermvs")
        else()
            # Keep deterministic paths for diagnostics/cache editing even when
            # auto-detection did not find the SDK.
            set(PCEDITOR_EIGEN3_ROOT "")
            set(PCEDITOR_OPENCV_ROOT "")
            set(PCEDITOR_RULERMVS_ROOT "")
        endif()

        set(_PCEDITOR_RULERMVS_ENABLED FALSE)
        if(WIN32 AND THIRD_PATH)
            find_path(PCEDITOR_RULERMVS_INCLUDE_DIR
                NAMES rulermvs.hpp
                PATHS
                    "${PCEDITOR_RULERMVS_ROOT}/include"
                    "${PCEDITOR_RULERMVS_ROOT}"
                NO_DEFAULT_PATH)
            find_path(PCEDITOR_DBOW3_INCLUDE_DIR
                NAMES DBoW3/DBoW3.h
                PATHS
                    "${PCEDITOR_RULERMVS_ROOT}/include"
                    "${PCEDITOR_RULERMVS_ROOT}/include/DBoW3"
                    "${PCEDITOR_RULERMVS_ROOT}"
                NO_DEFAULT_PATH)
            find_path(PCEDITOR_RULERMVS_LIB_DIR
                NAMES rulermvs.lib rulerMVS.lib
                PATHS
                    "${PCEDITOR_RULERMVS_ROOT}/lib"
                    "${PCEDITOR_RULERMVS_ROOT}/lib/x64"
                    "${PCEDITOR_RULERMVS_ROOT}/x64/lib"
                NO_DEFAULT_PATH)

            if(NOT PCEDITOR_RULERMVS_INCLUDE_DIR OR NOT PCEDITOR_DBOW3_INCLUDE_DIR)
                message(WARNING
                    "rulermvs headers are incomplete under ${PCEDITOR_RULERMVS_ROOT}; "
                    "rulermvs.hpp=${PCEDITOR_RULERMVS_INCLUDE_DIR}, DBoW3=${PCEDITOR_DBOW3_INCLUDE_DIR}")
            elseif(NOT PCEDITOR_RULERMVS_LIB_DIR)
                # Some SDK packages use non-standard import library names.  If
                # a lib directory exists, accept it and glob the import libs below.
                foreach(_lib_candidate
                        "${PCEDITOR_RULERMVS_ROOT}/lib"
                        "${PCEDITOR_RULERMVS_ROOT}/lib/x64"
                        "${PCEDITOR_RULERMVS_ROOT}/x64/lib")
                    if(EXISTS "${_lib_candidate}")
                        set(PCEDITOR_RULERMVS_LIB_DIR "${_lib_candidate}")
                        break()
                    endif()
                endforeach()
            endif()
        endif()

        if(WIN32
           AND THIRD_PATH
           AND PCEDITOR_RULERMVS_INCLUDE_DIR
           AND PCEDITOR_DBOW3_INCLUDE_DIR
           AND PCEDITOR_RULERMVS_LIB_DIR)

            # Eigen3 is header-only. Support common directory layouts.
            find_path(PCEDITOR_EIGEN3_INCLUDE_DIR
                NAMES Eigen/Core
                PATHS
                    "${PCEDITOR_EIGEN3_ROOT}"
                    "${PCEDITOR_EIGEN3_ROOT}/include"
                    "${PCEDITOR_EIGEN3_ROOT}/include/eigen3"
                NO_DEFAULT_PATH)

            if(NOT PCEDITOR_EIGEN3_INCLUDE_DIR)
                message(FATAL_ERROR
                    "Eigen3 was not found under ${PCEDITOR_EIGEN3_ROOT}. "
                    "Expected Eigen/Core in eigen3, eigen3/include, or eigen3/include/eigen3.")
            endif()

            # ---------------------------------------------------------
            # OpenCV 4.8.0 - manual include/lib configuration
            # ---------------------------------------------------------
            find_path(PCEDITOR_OPENCV_INCLUDE_DIR
                NAMES opencv2/core.hpp
                PATHS
                    "${PCEDITOR_OPENCV_ROOT}/include"
                    "${PCEDITOR_OPENCV_ROOT}/build/include"
                NO_DEFAULT_PATH)
            if(NOT PCEDITOR_OPENCV_INCLUDE_DIR)
                message(FATAL_ERROR
                    "OpenCV headers were not found under ${PCEDITOR_OPENCV_ROOT}")
            endif()
            set(OpenCV_INCLUDE_DIRS "${PCEDITOR_OPENCV_INCLUDE_DIR}")

            # Locate the prebuilt OpenCV import-library directory.  Support the
            # old opencv/lib layout and official opencv/x64/vc*/lib packages.
            set(PCEDITOR_OPENCV_LIB_DIR "")
            foreach(_opencv_lib_candidate
                    "${PCEDITOR_OPENCV_ROOT}/lib")
                if(EXISTS "${_opencv_lib_candidate}/opencv_core480.lib")
                    set(PCEDITOR_OPENCV_LIB_DIR "${_opencv_lib_candidate}")
                    break()
                endif()
            endforeach()
            if(NOT PCEDITOR_OPENCV_LIB_DIR)
                file(GLOB _opencv_vc_lib_dirs LIST_DIRECTORIES TRUE
                    "${PCEDITOR_OPENCV_ROOT}/x64/vc*/lib")
                foreach(_opencv_lib_candidate IN LISTS _opencv_vc_lib_dirs)
                    if(EXISTS "${_opencv_lib_candidate}/opencv_core480.lib")
                        set(PCEDITOR_OPENCV_LIB_DIR "${_opencv_lib_candidate}")
                        break()
                    endif()
                endforeach()
            endif()
            if(NOT PCEDITOR_OPENCV_LIB_DIR)
                message(FATAL_ERROR
                    "OpenCV 4.8 import libraries were not found under ${PCEDITOR_OPENCV_ROOT}")
            endif()

            set(OpenCV_LIBS
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_core480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_imgcodecs480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_highgui480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_imgproc480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudaarithm480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudabgsegm480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudacodec480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudafeatures2d480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudafilters480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudaimgproc480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudalegacy480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudaobjdetect480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudaoptflow480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudastereo480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudawarping480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_cudev480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_dnn_objdetect480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_dnn_superres480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_dnn480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_video480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_ccalib480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_videoio480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_flann480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_features2d480.lib
                ${PCEDITOR_OPENCV_LIB_DIR}/opencv_calib3d480.lib
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
            file(GLOB _PCEDITOR_RULERMVS_LIBS CONFIGURE_DEPENDS
                "${PCEDITOR_RULERMVS_LIB_DIR}/*.lib")
            if(NOT _PCEDITOR_RULERMVS_LIBS)
                message(FATAL_ERROR
                    "No rulermvs .lib files were found in ${PCEDITOR_RULERMVS_LIB_DIR}")
            endif()

            target_include_directories(pceditor_qt_editor PRIVATE
                "${PCEDITOR_RULERMVS_INCLUDE_DIR}"
                "${PCEDITOR_DBOW3_INCLUDE_DIR}"
                "${PCEDITOR_EIGEN3_INCLUDE_DIR}"
                ${OpenCV_INCLUDE_DIRS}
            )

            target_link_libraries(pceditor_qt_editor PRIVATE
                ${_PCEDITOR_RULERMVS_LIBS}
                ${OpenCV_LIBS}
                strmiids ole32 oleaut32
            )

            target_compile_definitions(pceditor_qt_editor PRIVATE PCEDITOR_HAS_RULERMVS=1)
            set(_PCEDITOR_RULERMVS_ENABLED TRUE)

            message(STATUS "PointCloudEditor rulermvs scan integration: enabled")
            message(STATUS "  THIRD_PATH: ${THIRD_PATH}")
            message(STATUS "  rulermvs:   ${PCEDITOR_RULERMVS_ROOT}")
            message(STATUS "  OpenCV:     ${PCEDITOR_OPENCV_ROOT}")
            message(STATUS "  Eigen3:     ${PCEDITOR_EIGEN3_INCLUDE_DIR}")

            # Runtime deployment: copy rulermvs DLLs beside pceditor_qt_editor.exe.
            file(GLOB _PCEDITOR_RULERMVS_DLLS CONFIGURE_DEPENDS
                "${PCEDITOR_RULERMVS_ROOT}/bin/*.dll")
            if(_PCEDITOR_RULERMVS_DLLS)
                add_custom_command(TARGET pceditor_qt_editor POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            ${_PCEDITOR_RULERMVS_DLLS}
                            "$<TARGET_FILE_DIR:pceditor_qt_editor>"
                    COMMENT "Copying rulermvs runtime DLLs"
                    COMMAND_EXPAND_LISTS
                    VERBATIM)
            endif()

            # Copy bundled OpenCV runtime DLLs when present.
            # Common layouts supported:
            #   opencv/bin/*.dll
            #   opencv/x64/vc*/bin/*.dll
            file(GLOB_RECURSE _PCEDITOR_OPENCV_DLLS CONFIGURE_DEPENDS
                "${PCEDITOR_OPENCV_ROOT}/bin/*.dll"
                "${PCEDITOR_OPENCV_ROOT}/x64/*/bin/*.dll")
            if(_PCEDITOR_OPENCV_DLLS)
                add_custom_command(TARGET pceditor_qt_editor POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            ${_PCEDITOR_OPENCV_DLLS}
                            "$<TARGET_FILE_DIR:pceditor_qt_editor>"
                    COMMENT "Copying OpenCV runtime DLLs"
                    COMMAND_EXPAND_LISTS
                    VERBATIM)
            endif()

            # Camera VID/PID -> model profile used by the scan UI and capture worker.
            add_custom_command(TARGET pceditor_qt_editor POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                        "$<TARGET_FILE_DIR:pceditor_qt_editor>/config"
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${CMAKE_CURRENT_SOURCE_DIR}/examples/qt_editor/config/camera_models.json"
                        "$<TARGET_FILE_DIR:pceditor_qt_editor>/config/camera_models.json"
                COMMENT "Copying camera model configuration"
                VERBATIM)
        endif()

        if(WIN32 AND NOT _PCEDITOR_RULERMVS_ENABLED)
            message(WARNING
                "PointCloudEditor rulermvs scan integration: disabled. "
                "Set -DPCEDITOR_WIN32_THIRD_PARTY_ROOT=<path> to the directory "
                "containing eigen3, opencv and rulermvs. Auto-probed: "
                "${_PCEDITOR_WIN32_THIRD_CANDIDATES}")
        endif()

        if(_PCEDITOR_SELECTED_BACKEND STREQUAL "GLES31")
            target_compile_definitions(pceditor_render PRIVATE PCEDITOR_RENDER_GLES31=1)
            target_compile_definitions(pceditor_qt_editor PRIVATE PCEDITOR_RENDER_GLES31=1)
        elseif(_PCEDITOR_SELECTED_BACKEND STREQUAL "DESKTOP_GL21")
            target_compile_definitions(pceditor_render PRIVATE PCEDITOR_RENDER_DESKTOP_GL21=1)
            target_compile_definitions(pceditor_qt_editor PRIVATE PCEDITOR_RENDER_DESKTOP_GL21=1)
        else()
            message(FATAL_ERROR "Unknown PCEDITOR_RENDER_BACKEND=${_PCEDITOR_SELECTED_BACKEND}")
        endif()

        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/examples" PREFIX "Example Files" FILES
            examples/qt_editor/main.cpp
            examples/qt_editor/MainWindow.cpp
            examples/qt_editor/MainWindow.h
            examples/qt_editor/PointCloudWidget.cpp
            examples/qt_editor/PointCloudWidget.h
            examples/qt_editor/ProcessingDialog.cpp
            examples/qt_editor/ProcessingDialog.h
            examples/qt_editor/ScanFlowController.cpp
            examples/qt_editor/ScanFlowController.h
            examples/qt_editor/CameraDeviceManager.cpp
            examples/qt_editor/CameraDeviceManager.h
            examples/qt_editor/config/camera_models.json
            ${PCEDITOR_EXAMPLE_COMMON})
        set_target_properties(pceditor_qt_editor PROPERTIES FOLDER "Examples/Qt")

        # Windows 下自动部署 Qt + MSVC Runtime。
        if(WIN32)
            get_target_property(_PCEDITOR_QMAKE_EXECUTABLE Qt6::qmake IMPORTED_LOCATION)
            if(_PCEDITOR_QMAKE_EXECUTABLE)
                get_filename_component(_PCEDITOR_QT_BIN_DIR "${_PCEDITOR_QMAKE_EXECUTABLE}" DIRECTORY)
                find_program(PCEDITOR_WINDEPLOYQT_EXECUTABLE
                    NAMES windeployqt windeployqt.exe
                    HINTS "${_PCEDITOR_QT_BIN_DIR}"
                    NO_DEFAULT_PATH)
            endif()
            if(NOT PCEDITOR_WINDEPLOYQT_EXECUTABLE)
                find_program(PCEDITOR_WINDEPLOYQT_EXECUTABLE NAMES windeployqt windeployqt.exe)
            endif()
            if(PCEDITOR_WINDEPLOYQT_EXECUTABLE)
                add_custom_command(TARGET pceditor_qt_editor POST_BUILD
                    COMMAND "${PCEDITOR_WINDEPLOYQT_EXECUTABLE}"
                            --compiler-runtime --no-translations
                            --dir "$<TARGET_FILE_DIR:pceditor_qt_editor>"
                            "$<TARGET_FILE:pceditor_qt_editor>"
                    COMMENT "Deploying Qt and MSVC runtime dependencies with windeployqt"
                    VERBATIM)
            else()
                message(WARNING "windeployqt not found; Qt runtime files will not be copied automatically")
            endif()
        endif()

        if(ANDROID)
            qt_finalize_executable(pceditor_qt_editor)
            message(STATUS "PointCloudEditor Android APK target configured (GLES 3.1, package com.pceditor.viewer)")
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
if(WIN32 AND PCEDITOR_BUILD_WINDOWS_OPENGL_EXAMPLE)
    add_executable(pceditor_windows_opengl
        examples/windows_opengl/main.cpp
        ${PCEDITOR_EXAMPLE_COMMON}
    )
    target_link_libraries(pceditor_windows_opengl PRIVATE pceditor opengl32)
    target_compile_definitions(pceditor_windows_opengl PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/examples" PREFIX "Example Files" FILES
        examples/windows_opengl/main.cpp ${PCEDITOR_EXAMPLE_COMMON})
    set_target_properties(pceditor_windows_opengl PROPERTIES FOLDER "Examples/Windows")
endif()

# -----------------------------
# RK3588 / Linux EGL + GLES3 + R32UI GPU Picking
# -----------------------------
# 只有检测到 EGL/GLES3 开发文件后才生成目标，避免普通 PC 环境配置失败。
if(UNIX AND NOT APPLE AND PCEDITOR_BUILD_RK3588_GLES_EXAMPLE)
    find_path(PCEDITOR_EGL_INCLUDE_DIR EGL/egl.h)
    find_path(PCEDITOR_GLES3_INCLUDE_DIR GLES3/gl3.h)
    find_library(PCEDITOR_EGL_LIBRARY EGL)
    find_library(PCEDITOR_GLES3_LIBRARY GLESv2)
    if(PCEDITOR_EGL_INCLUDE_DIR AND PCEDITOR_GLES3_INCLUDE_DIR AND PCEDITOR_EGL_LIBRARY AND PCEDITOR_GLES3_LIBRARY)
        add_executable(pceditor_rk3588_gles
            examples/rk3588_gles/main.cpp
            ${PCEDITOR_EXAMPLE_COMMON}
        )
        target_include_directories(pceditor_rk3588_gles PRIVATE
            ${PCEDITOR_EGL_INCLUDE_DIR} ${PCEDITOR_GLES3_INCLUDE_DIR})
        target_link_libraries(pceditor_rk3588_gles PRIVATE
            pceditor ${PCEDITOR_EGL_LIBRARY} ${PCEDITOR_GLES3_LIBRARY})
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/examples" PREFIX "Example Files" FILES
            examples/rk3588_gles/main.cpp ${PCEDITOR_EXAMPLE_COMMON})
        set_target_properties(pceditor_rk3588_gles PROPERTIES FOLDER "Examples/RK3588")
    else()
        message(STATUS "EGL/GLES3 development files not found; skipping RK3588 GLES example")
    endif()
endif()

# -----------------------------
# 安装规则
# -----------------------------
install(TARGETS pceditor
    EXPORT PointCloudEditorTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
if(TARGET pceditor_render)
    install(TARGETS pceditor_render
        EXPORT PointCloudEditorTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()
install(DIRECTORY include/ DESTINATION include)

# Keep the official SurfaceTrimmer beside runtime editors/tools as a deployment fallback.
if(TARGET pceditor_qt_editor AND TARGET pceditor_surface_trimmer_official)
    add_custom_command(TARGET pceditor_qt_editor POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "$<TARGET_FILE:pceditor_surface_trimmer_official>"
                "$<TARGET_FILE_DIR:pceditor_qt_editor>/$<TARGET_FILE_NAME:pceditor_surface_trimmer_official>"
        COMMENT "Copying official PoissonRecon SurfaceTrimmer beside pceditor_qt_editor"
        VERBATIM)
endif()
if(TARGET pceditor_surface_trimmer_official)
    install(TARGETS pceditor_surface_trimmer_official RUNTIME DESTINATION bin)
endif()
