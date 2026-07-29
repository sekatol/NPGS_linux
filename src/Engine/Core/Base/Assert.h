#pragma once

#ifdef _DEBUG
#define NPGS_ENABLE_ASSERT
#endif

#ifdef NPGS_ENABLE_ASSERT
#include <iostream>

#if defined(_WIN32)
#include <Windows.h>
#define NPGS_DEBUG_BREAK() DebugBreak()
#elif defined(__linux__)
#include <csignal>
#define NPGS_DEBUG_BREAK() raise(SIGTRAP)
#endif

#define NpgsAssert(Expr, ...)                                                                                 \
if (!(Expr))                                                                                                  \
{                                                                                                             \
    std::cerr << "Assertion failed: " << #Expr << " in " << __FILE__ << " at line " << __LINE__ << std::endl; \
    std::cerr << "Message: " << __VA_ARGS__ << std::endl;                                                     \
    NPGS_DEBUG_BREAK();                                                                                       \
}

#define NpgsStaticAssert(Expr, ...) static_assert(Expr, __VA_ARGS__)

#else

#define NpgsAssert(Expr, ...)       static_cast<void>(0)
#define NpgsStaticAssert(Expr, ...) static_cast<void>(0)

#endif
