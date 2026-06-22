
#include "DebugUtils.h"
 
// Static member definitions — required so the linker finds storage for these.
// (One definition per TU; do not move these into the header.)
 
std::chrono::time_point<std::chrono::high_resolution_clock> DebugUtils::start_time;
std::chrono::time_point<std::chrono::high_resolution_clock> DebugUtils::end_time;
std::ofstream DebugUtils::log_file;
std::mutex DebugUtils::log_mutex;

std::unordered_map<std::string, const char*> DebugUtils::tag_colors = {
    { "ERROR",     LogColor::RED     },
    { "WARNING",   LogColor::YELLOW  },
    { "DEBUG",     LogColor::BLUE    },
    { "INFO",      LogColor::CYAN    },
    { "CONFIG",    LogColor::GREEN   },
    { "SAVE",      LogColor::MAGENTA },
    { "THREAD",    LogColor::WHITE   },
    // Tags not listed here will default to no color
};