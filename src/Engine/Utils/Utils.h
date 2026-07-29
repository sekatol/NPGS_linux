#pragma once

#include "Engine/Core/Base/Base.h"

_NPGS_BEGIN
_UTIL_BEGIN

bool Equal(const char* Lhs, const char* Rhs);
bool Equal(float Lhs, float Rhs);
bool Equal(double Lhs, double Rhs);

_UTIL_END
_NPGS_END

#include "Utils.inl"
