#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "Engine/Utils/Logger.h"

_NPGS_BEGIN
_UTIL_BEGIN

std::shared_ptr<spdlog::logger> FLogger::_kCoreLogger;
std::shared_ptr<spdlog::logger> FLogger::_kClientLogger;

void FLogger::Initialize()
{
#if defined(NPGS_ENABLE_CONSOLE_LOGGER)
    _kCoreLogger = spdlog::stdout_color_mt("NPGS");
    _kCoreLogger->set_level(spdlog::level::trace);
    _kCoreLogger->set_pattern("%^[%T] %n: %v%$");

    _kClientLogger = spdlog::stdout_color_mt("App");
    _kClientLogger->set_level(spdlog::level::trace);
    _kClientLogger->set_pattern("%^[%T] %n: %v%$");
#elif defined(NPGS_ENABLE_FILE_LOGGER)
    _kCoreLogger = spdlog::basic_logger_mt("NPGS", "NpgsCore.log");
    _kCoreLogger->set_level(spdlog::level::info);
    _kCoreLogger->set_pattern("[%T] %n: %v");

    _kClientLogger = spdlog::basic_logger_mt("App", "Npgs.log");
    _kClientLogger->set_level(spdlog::level::info);
    _kClientLogger->set_pattern("[%T] %n: %v");
#endif
}

_UTIL_END
_NPGS_END
