#pragma once

#include <utility>

#ifdef _RELEASE
#define RELEASE_FORCE_INLINE
#endif

#ifdef _MSC_VER
#   ifdef RELEASE_FORCE_INLINE
#       define NPGS_INLINE __forceinline
#   else
#       define NPGS_INLINE inline
#   endif
#elif defined(__GNUC__) || defined(__clang__)
#   ifdef RELEASE_FORCE_INLINE
#       define NPGS_INLINE __attribute__((always_inline)) inline
#   else
#       define NPGS_INLINE inline
#   endif
#else
#   error Unsupported compiler
#endif

#define _ASSET_BEGIN namespace Asset {
#define _ASSET_END }
#define _ASTRO_BEGIN namespace Astro {
#define _ASTRO_END }
#define _CONFIG_BEGIN namespace Config {
#define _CONFIG_END }
#define _GENERATOR_BEGIN namespace Generator {
#define _GENERATOR_END }
#define _GRAPHICS_BEGIN namespace Graphics {
#define _GRAPHICS_END }
#define _INTELLI_BEGIN namespace Intelli {
#define _INTELLI_END }
#define _MATH_BEGIN namespace Math {
#define _MATH_END }
#define _NPGS_BEGIN namespace Npgs {
#define _NPGS_END }
#define _RUNTIME_BEGIN namespace Runtime {
#define _RUNTIME_END }
#define _SPATIAL_BEGIN namespace Spatial {
#define _SPATIAL_END }
#define _UI_BEGIN namespace UI {
#define _UI_END }
#define _SYSTEM_BEGIN namespace System {
#define _SYSTEM_END }
#define _THREAD_BEGIN namespace Thread {
#define _THREAD_END }
#define _UTIL_BEGIN namespace Util {
#define _UTIL_END }

#define Bit(x) (1ULL << x)

#define NpgsBindMemberFunc(Func) [this](auto&&... Args) -> decltype(auto) \
{                                                                         \
    return this->Func(std::forward<decltype(Args)>(Args)...);             \
}

#define NpgsBind(Func) [](auto&&... Args) -> decltype(auto) \
{                                                           \
    return Func(std::forward<decltype(Args)>(Args)...);     \
}
