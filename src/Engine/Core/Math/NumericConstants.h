#pragma once

#include <cstdint>

namespace Npgs::Math
{

constexpr double kPi          = 3.14159265358979323846;
constexpr double kTwoPi       = 2.0 * kPi;
constexpr double kHalfPi      = kPi / 2.0;
constexpr double kQuarterPi   = kPi / 4.0;
constexpr double kInvPi       = 1.0 / kPi;
constexpr double kDegToRad    = kPi / 180.0;
constexpr double kRadToDeg    = 180.0 / kPi;
constexpr double kEpsilon     = 1e-10;
constexpr double kInfinity    = 1e30;

constexpr std::uint64_t kKibiByte = 1024ULL;
constexpr std::uint64_t kMebiByte = 1024ULL * kKibiByte;
constexpr std::uint64_t kGibiByte = 1024ULL * kMebiByte;
constexpr std::uint64_t kTebiByte = 1024ULL * kGibiByte;

constexpr double kGravitationalConstant   = 6.67430e-11;
constexpr double kGravityConstant         = kGravitationalConstant;
constexpr double kSpeedOfLight            = 299792458.0;
constexpr double kSolarMass               = 1.98847e30;
constexpr double kSolarRadius             = 6.9634e8;
constexpr double kSolarLuminosity         = 3.828e26;
constexpr double kAstronomicalUnit        = 1.495978707e11;
constexpr double kParsec                  = 3.085677581e16;
constexpr double kLightYear               = 9.4607304725808e15;
constexpr double kLightYearToMeter        = kLightYear;

}
