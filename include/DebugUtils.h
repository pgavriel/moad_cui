/*
    8/26 added DebugUtils to some files, made more modular with more functionality
        continuing to implement within other source files

    GS 8/26

    as of 8/12, this is just being used in RealSenseHandler but it SEEMS like
        it can be used in all other locations.
    
    i cannot tell if the functionality was taken from realsense related code
        or if it was written by hand, however this SHOULDNT be an issue, unless the
        instantiation code i assume for singleton instances doesnt work with
        other parts of the code. remember that this code uses singleton, multithreading
        and global variables in different places.
  
    GS 8/12

    Refactored all log*() and print*() wrappers to call a single master log() function.
    log() checks "log_debug" config to write to file, and "debug_verbosity" config to
    optionally echo to terminal. Each wrapper passes a fixed tag and a default debug
    level; callers can also call log() directly with a custom tag and level.

    "always_log" behavior (logInfo, logWarning, logConfig) is preserved — these write
    to the log file regardless of the "log_debug" config flag.
*/

#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <fstream>
#include <ctime>
#include <cstdio>
#include <mutex>

#include "ConfigHandler.h"

// ANSI terminal color codes — only ever applied to terminal output, never logged to file
// Associations between log message tags and colors are defined in DebugUtils.cpp
namespace LogColor {
    // Reset
    constexpr const char* RESET          = "\033[0m";

    // Standard foreground colors
    constexpr const char* RED            = "\033[31m";
    constexpr const char* YELLOW         = "\033[33m";
    constexpr const char* GREEN          = "\033[32m";
    constexpr const char* CYAN           = "\033[36m";
    constexpr const char* MAGENTA        = "\033[35m";
    constexpr const char* BLUE           = "\033[34m";
    constexpr const char* WHITE          = "\033[37m";

    // Bright/bold foreground — noticeably more vivid than the standard variants
    constexpr const char* BRIGHT_RED     = "\033[91m";
    constexpr const char* BRIGHT_YELLOW  = "\033[93m";
    constexpr const char* BRIGHT_GREEN   = "\033[92m";
    constexpr const char* BRIGHT_CYAN    = "\033[96m";
    constexpr const char* BRIGHT_MAGENTA = "\033[95m";
    constexpr const char* BRIGHT_BLUE    = "\033[94m";
    constexpr const char* BRIGHT_WHITE   = "\033[97m";
    constexpr const char* ORANGE         = "\033[38;5;214m";  // 256-color orange
    constexpr const char* PINK           = "\033[38;5;213m";  // 256-color pink
    constexpr const char* TEAL           = "\033[38;5;38m";   // 256-color teal

    // Bold modifier — combine with a color for emphasis
    constexpr const char* BOLD           = "\033[1m";
    constexpr const char* BOLD_RED       = "\033[1;31m";
    constexpr const char* BOLD_YELLOW    = "\033[1;33m";
    constexpr const char* BOLD_GREEN     = "\033[1;32m";
    constexpr const char* BOLD_CYAN      = "\033[1;36m";
    constexpr const char* BOLD_WHITE     = "\033[1;37m";

    // Background highlights — useful for tags you really want to stand out
    constexpr const char* MAGENTA_BKG    = "\033[1;45m";
    constexpr const char* RED_BKG        = "\033[1;41m";
    constexpr const char* YELLOW_BKG     = "\033[1;43m";
    constexpr const char* GREEN_BKG      = "\033[1;42m";
    constexpr const char* CYAN_BKG       = "\033[1;46m";
    constexpr const char* BLUE_BKG       = "\033[1;44m";
}

class DebugUtils {
private:
    DebugUtils() {}
    ~DebugUtils() {}
    DebugUtils(const DebugUtils&) = delete;
    DebugUtils& operator=(const DebugUtils&) = delete;

    static std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    static std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    static std::ofstream log_file;
    static std::mutex log_mutex;
    static std::unordered_map<std::string, const char*> tag_colors;
    static bool config_available;
    static std::string log_path;

public:

    // -------------------------------------------------------------------------
    // Log file lifecycle
    // -------------------------------------------------------------------------

    // Start a new log file at file_path. If a log file is currently open,
    // it is safely closed with an end message before the new one is opened.
    // Safe to call even if ConfigHandler is not yet initialised — in that
    // case the file write is skipped and only terminal output occurs.
    static void startLogFile(const std::string& file_path) {
        // Safely close any currently open log file first
        closeLogFile();

        // If config is available, respect the "debug.log" flag
        if (_configAvailable()) {
            ConfigHandler& config = ConfigHandler::getInstance();
            if (!config.getValue<bool>("debug.log")) {
                return;
            }
        }

        log_file.open(file_path, std::ios::out | std::ios::app);
        if (!log_file.is_open()) {
            fprintf(stderr, "[ERROR] Failed to open log file: %s\n", file_path.c_str());
            return;
        }
        // Store log path to print when closing.
        log_path = file_path;

        // Write startup timestamp entry
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm;
        localtime_r(&now_time, &local_tm);
        char date_buf[32], time_buf[32];
        std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", &local_tm);
        std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &local_tm);

        log("START", std::string("Debug logging started..."
            "\n\tlog:  ") + file_path +
            "\n\tdate: " + date_buf +
            "\n\ttime: " + time_buf, 0);
        logWhitespace();
    }

    // Keep initLogFile() as a backwards-compatible alias so existing call sites
    // don't need to change.
    static void initLogFile(const std::string& file_path) {
        startLogFile(file_path);
    }

    static void closeLogFile() {
    if (log_file.is_open()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm;
        localtime_r(&now_time, &local_tm);
        char time_buf[32];
        std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &local_tm);

        log("END", std::string("Closing log at: ") + log_path + "\n\n", 0);
        log_file.close();
        log("END", std::string("Debug logging ended at ") + time_buf + "\n", 0);
    }
}


    // -------------------------------------------------------------------------
    // Master log function
    //
    //   tag         - label written to the log line, e.g. "DEBUG", "CAMERA"
    //   message     - the log message
    //   debug_level - verbosity level of this message (default 0).
    //                 The message is echoed to the terminal only when:
    //                   config["debug_verbosity"] >= debug_level
    //   always_log  - when true, write to the file even if "log_debug" is false
    //                 (used by logInfo, logWarning, logConfig to preserve their
    //                  original bypass behaviour)
    //   TODO: Implement text color choices for more organized printing
    // -------------------------------------------------------------------------

    static void log(const std::string& tag,
                    const std::string& message,
                    int  debug_level = 0,
                    bool always_log  = false)
    {
        // Mutex lock for threadsafe logging/printing
        std::lock_guard<std::mutex> lock(log_mutex);

        // Obtain global config
        // ConfigHandler& config = ConfigHandler::getInstance();

        // Fixed width formatting for debug tags:
        static constexpr int TAG_WIDTH = 9; // SET MAX TAG WIDTH
        char tag_buf[TAG_WIDTH + 1];
        int tag_len = static_cast<int>(tag.size());
        int total_pad = TAG_WIDTH - tag_len;
        int pad_left  = total_pad / 2;
        int pad_right = total_pad - pad_left;  // takes the extra space if odd
        std::snprintf(tag_buf, sizeof(tag_buf), "%*s%s%*s",
                    pad_left, "", tag.c_str(), pad_right, "");

        // FILE WRITE — skipped entirely if config isn't ready yet
        if (_configAvailable()) {
            // Obtain global config
            ConfigHandler& config = ConfigHandler::getInstance();
            const bool file_enabled = always_log || config.getValue<bool>("debug.log");
            if (file_enabled && log_file.is_open()) {
                log_file << "[" << tag_buf << "][" << debug_level << "] " << message << std::endl;
            }
        }

        // TERMINAL PRINT — always runs; verbosity gated only if config is ready
        // If config isn't ready yet, print everything.
        bool should_print = true;
        if (_configAvailable()) {
            const int verbosity = ConfigHandler::getInstance().getValue<int>("debug.debug_verbosity");
            should_print = (verbosity >= debug_level);
        }

        if (should_print) {
            auto it = tag_colors.find(tag);
            const char* color = (it != tag_colors.end()) ? it->second : "";
            const char* reset = (it != tag_colors.end()) ? LogColor::RESET : "";
            if (tag == "ERROR") {
                fprintf(stderr, "%s[%s][%d] %s%s\n", color, tag_buf, debug_level, message.c_str(), reset);
            } else {
                fprintf(stdout, "%s[%s][%d] %s%s\n", color, tag_buf, debug_level, message.c_str(), reset);
            }
        }
    }

    // Call this once ConfigHandler has been successfully initialised.
    // Until then, log() prints to terminal at all verbosity levels and
    // skips all file writes.
    static void notifyConfigReady(bool ready = true) {
        config_available = ready;
    }

    // -------------------------------------------------------------------------
    // Named wrappers — preserved so existing call sites need no changes
    // -------------------------------------------------------------------------

    // debug_level 0 — general debug noise, shown at any verbosity
    static void logDebug(const std::string& message) {
        log("DEBUG", message, 0);
    }

    // debug_level 1 — threading events, slightly more verbose
    static void logThread(const std::string& message) {
        log("THREAD", message, 1);
    }

    // always_log = true — config check was intentionally bypassed in original
    static void logConfig(const std::string& message) {
        log("CONFIG", message, 0, /*always_log=*/true);
    }

    static void logCam(const std::string& message) {
        log("CAMERA", message, 0);
    }

    static void logFileSys(const std::string& message){
        log("FILESYS", message, 0);
    }

    static void logRS(const std::string& message, int v_lvl = 0) {
        log("REALSENSE", message, v_lvl);
    }

    static void logCanon(const std::string& message, int v_lvl = 0) {
        log("CANON", message, v_lvl);
    }

    static void logError(const std::string& message) {
        log("ERROR", message, 0);
    }

    static void logEnd(const std::string& message) {
        log("END", message, 0);
    }

    static void logPictureLoop(const std::string& message) {
        log("TAKE", message, 1);
    }

    static void logSaveLoop(const std::string& message) {
        log("SAVE", message, 0);
    }

    static void logTurntable(const std::string& message, int v_lvl = 1) {
        log("TURNTABLE", message, v_lvl);
    }

    // always_log = true — original logInfo/logWarning had no config guard
    static void logInfo(const std::string& message) {
        log("INFO", message, 0, /*always_log=*/true);
    }

    static void logWarning(const std::string& message) {
        log("WARNING", message, 0, /*always_log=*/true);
    }

    static void logSerial(const std::string& message, int v_lvl = 0) {
        log("SERIAL", message, v_lvl);
    }

    static void logWhitespace(int n = 1) {
    std::lock_guard<std::mutex> lock(log_mutex);
    for (int i = 0; i < n; i++) {
        fprintf(stdout, "\n");
    }
}

    // -------------------------------------------------------------------------
    // Timer helpers
    // -------------------------------------------------------------------------

    static void startTimer() {
        if (!_debugEnabled()) return;
        start_time = std::chrono::high_resolution_clock::now();
    }

    // TODO: There is only one start time, so parallel timers won't work at all.
    static void stopTimer(const std::string& message) {
        if (!_debugEnabled()) return;
        end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::stringstream ss;
        ss << message << " took " << duration.count() << " ms";
        log("TIMER", ss.str(), 3);
    }


private:

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    static bool _debugEnabled() {
        if (!_configAvailable()) return false;
        return ConfigHandler::getInstance().getValue<bool>("debug.log");
    }

    static bool _configAvailable() {
    return config_available;
}
};