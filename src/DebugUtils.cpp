#include "DebugUtils.h"

// these need to be here to make sure memory is allocated for static variables (i think) - GS 8/12

std::chrono::time_point<std::chrono::high_resolution_clock> DebugUtils::start_time;
std::chrono::time_point<std::chrono::high_resolution_clock> DebugUtils::end_time;
std::ofstream DebugUtils::log_file;