
#include "DebugUtils.h"
#include <unordered_map>
 
// Static member definitions — required so the linker finds storage for these.
// (One definition per TU; do not move these into the header.)
 
std::chrono::time_point<std::chrono::high_resolution_clock> DebugUtils::start_time;
std::chrono::time_point<std::chrono::high_resolution_clock> DebugUtils::end_time;
std::ofstream DebugUtils::log_file;
std::mutex DebugUtils::log_mutex;
bool DebugUtils::config_available = false;
std::string DebugUtils::log_path;


std::unordered_map<std::string, const char*> DebugUtils::tag_colors = {
    { "ERROR",     LogColor::RED_BKG       },  // hard to miss
    { "WARNING",   LogColor::BOLD_YELLOW   },  // stands out, less severe than ERROR
    { "INFO",      LogColor::BLUE_BKG   },
    { "DEBUG",     LogColor::BLUE          },  // low-key, common noise
    { "CONFIG",    LogColor::BRIGHT_GREEN  },
    { "CAMERA",    LogColor::BRIGHT_MAGENTA},
    { "CANON",     LogColor::BRIGHT_MAGENTA},
    { "REALSENSE", LogColor::TEAL          },
    { "THREAD",    LogColor::WHITE        },
    { "SERIAL",    LogColor::ORANGE          },
    { "TURNTABLE", LogColor::BRIGHT_BLUE   },
    { "SAVE",      LogColor::MAGENTA       },
    { "TAKE",      LogColor::BRIGHT_WHITE  },
    { "FILESYS",   LogColor::BOLD_CYAN     },
    { "END",       LogColor::BOLD_WHITE    },
    // Tags not listed here will default to no color
};