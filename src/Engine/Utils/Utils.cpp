#include <cmath>
#include <cstring>
#include "Engine/Utils/Utils.h"
#include "Engine/Core/Math/NumericConstants.h"

_NPGS_BEGIN
_UTIL_BEGIN

bool Equal(const char* Lhs, const char* Rhs)
{
    return std::strcmp(Lhs, Rhs) == 0;
}

bool Equal(float Lhs, float Rhs)
{
    return std::fabs(Lhs - Rhs) < Math::kEpsilon;
}

bool Equal(double Lhs, double Rhs)
{
    return std::fabs(Lhs - Rhs) < Math::kEpsilon;
}

_UTIL_END
_NPGS_END
