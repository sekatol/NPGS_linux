cmake_minimum_required(VERSION 3.25)
project(NPGS_linux VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-attributes")
endif()

# -------------------------------------------------------------------
# Dependencies — system packages + FetchContent
# -------------------------------------------------------------------
find_package(Vulkan REQUIRED)
find_package(glfw3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(glm REQUIRED)
find_package(nlohmann_json REQUIRED)

# spirv-cross (Arch: no cmake/pkgconfig, static libs)
find_path(SPIRV_CROSS_INCLUDE_DIR spirv_cross/spirv_reflect.hpp
  PATHS /usr/include /usr/local/include REQUIRED)
find_library(SPIRV_CROSS_CORE_LIB spirv-cross-core
  PATHS /usr/lib /usr/local/lib
  REQUIRED)
find_library(SPIRV_CROSS_GLSL_LIB spirv-cross-glsl
  PATHS /usr/lib /usr/local/lib
  REQUIRED)
find_library(SPIRV_CROSS_REFLECT_LIB spirv-cross-reflect
  PATHS /usr/lib /usr/local/lib REQUIRED)

# Freetype via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREETYPE REQUIRED freetype2)

include(cmake/FetchDeps.cmake)

# -------------------------------------------------------------------
# Sources
# -------------------------------------------------------------------
set(IMGUI_SOURCES
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)

set(NPGS_SOURCES
  ${IMGUI_SOURCES}
  src/Program/main.cpp
  src/Program/Application.cpp

  src/Engine/Utils/Logger.cpp
  src/Engine/Utils/Utils.cpp

  src/Engine/Core/Runtime/Graphics/Vulkan/Core.cpp
  src/Engine/Core/Runtime/Graphics/Vulkan/Context.cpp
  src/Engine/Core/Runtime/Graphics/Vulkan/Wrappers.cpp
  src/Engine/Core/Runtime/Graphics/Vulkan/Resources.cpp
  src/Engine/Core/Runtime/Graphics/Vulkan/ShaderResourceManager.cpp
  src/Engine/Core/Runtime/Graphics/Vulkan/ExtFunctionsImpl.cpp
  src/Engine/Core/Runtime/Graphics/Vulkan/VulkanUIRenderer.cpp

  src/Engine/Core/Runtime/Graphics/Renderers/PipelineManager.cpp

  src/Engine/Core/Runtime/AssetLoaders/AssetManager.cpp
  src/Engine/Core/Runtime/AssetLoaders/Shader.cpp
  src/Engine/Core/Runtime/AssetLoaders/Texture.cpp
  src/Engine/Core/Runtime/AssetLoaders/GetAssetFullPath.cpp

  src/Engine/Core/System/Spatial/Camera.cpp

  src/ExternalImpl/stb_impl.cpp
  src/ExternalImpl/vma_impl.cpp
)

# -------------------------------------------------------------------
# Include paths
# -------------------------------------------------------------------
set(NPGS_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${Vulkan_INCLUDE_DIRS}
  ${glfw3_INCLUDE_DIRS}
  ${spdlog_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
  ${SPIRV_CROSS_INCLUDE_DIR}
  ${glm_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
  ${stb_SOURCE_DIR}
  ${vulkan-memory-allocator_SOURCE_DIR}/include
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
  ${magic_enum_SOURCE_DIR}/include
  ${fast-cpp-csv-parser_SOURCE_DIR}
)

# -------------------------------------------------------------------
# Executable
# -------------------------------------------------------------------
add_executable(NPGS_linux ${NPGS_SOURCES})

target_include_directories(NPGS_linux PRIVATE ${NPGS_INCLUDES})
target_precompile_headers(NPGS_linux PRIVATE src/stdafx.h)

target_link_libraries(NPGS_linux PRIVATE
  Vulkan::Vulkan
  glfw
  spdlog::spdlog_header_only
  ${FREETYPE_LIBRARIES}
  ${FREETYPE_LDFLAGS}
  ${SPIRV_CROSS_CORE_LIB}
  ${SPIRV_CROSS_GLSL_LIB}
  ${SPIRV_CROSS_REFLECT_LIB}
)

# -------------------------------------------------------------------
# Compile options
# -------------------------------------------------------------------
target_compile_definitions(NPGS_linux PRIVATE
  $<$<CONFIG:Debug>:NPGS_ENABLE_CONSOLE_LOGGER _DEBUG>
  $<$<CONFIG:Release>:NPGS_ENABLE_CONSOLE_LOGGER>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(NPGS_linux PRIVATE
    -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
    -Wno-unused-variable -Wno-sign-compare
  )
endif()

# -------------------------------------------------------------------
# Copy assets to build directory
# -------------------------------------------------------------------
file(COPY ${CMAKE_SOURCE_DIR}/assets/ DESTINATION ${CMAKE_BINARY_DIR}/assets/)
