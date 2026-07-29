_NPGS_BEGIN
_UTIL_BEGIN

inline std::shared_ptr<spdlog::logger>& FLogger::GetCoreLogger()
{
    return _kCoreLogger;
}

inline std::shared_ptr<spdlog::logger>& FLogger::GetClientLogger()
{
    return _kClientLogger;
}

_UTIL_END
_NPGS_END
