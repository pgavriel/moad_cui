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
namespace LogColor {
    constexpr const char* RESET   = "\033[0m";
    constexpr const char* RED     = "\033[31m";
    constexpr const char* YELLOW  = "\033[33m";
    constexpr const char* GREEN   = "\033[32m";
    constexpr const char* CYAN    = "\033[36m";
    constexpr const char* MAGENTA = "\033[35m";
    constexpr const char* BLUE    = "\033[34m";
    constexpr const char* WHITE   = "\033[37m";
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

public:

    // -------------------------------------------------------------------------
    // Log file lifecycle
    // -------------------------------------------------------------------------

    static void initLogFile(const std::string& file_path) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("debug.log")) {
            return;
        }

        log_file.open(file_path, std::ios::out | std::ios::app);
        if (!log_file.is_open()) {
            std::cerr << "[ERROR] Failed to open log file: " << file_path << std::endl;
        } else {
            // Build a timestamp string for the opening log entry
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm local_tm;
            localtime_r(&now_time, &local_tm);
            char date_buf[32], time_buf[32];
            std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", &local_tm);
            std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &local_tm);

            log("DEBUG", std::string("\n\n[START] Debug logging started,"
                "\n\tlog:  ") + file_path +
                "\n\tdate: " + date_buf +
                "\n\ttime: " + time_buf, 0);
        }
    }

    static void closeLogFile() {
    if (log_file.is_open()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm;
        localtime_r(&now_time, &local_tm);
        char time_buf[32];
        std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &local_tm);

        log("END", std::string("Debug logging ended at ") + time_buf + "\n\n\n", 0);
        log_file.close();
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
        ConfigHandler& config = ConfigHandler::getInstance();

        // Fixed width formatting for debug tags:
        static constexpr int TAG_WIDTH = 9; // SET MAX TAG WIDTH
        char tag_buf[TAG_WIDTH + 1];
        int tag_len = static_cast<int>(tag.size());
        int total_pad = TAG_WIDTH - tag_len;
        int pad_left  = total_pad / 2;
        int pad_right = total_pad - pad_left;  // takes the extra space if odd
        std::snprintf(tag_buf, sizeof(tag_buf), "%*s%s%*s",
                    pad_left, "", tag.c_str(), pad_right, "");

        // WRITE TO LOG FILE            
        const bool file_enabled = always_log || config.getValue<bool>("debug.log");
        if (file_enabled && log_file.is_open()) {
            log_file << "[" << tag_buf << "][" << debug_level << "] " << message << std::endl;
        }

        // PRINT LOG ITEMS TO TERMINAL
        const int verbosity = config.getValue<int>("debug.debug_verbosity");
        if (verbosity >= debug_level) {
            auto it = tag_colors.find(tag);
            const char* color = (it != tag_colors.end()) ? it->second : "";
            const char* reset = (it != tag_colors.end()) ? LogColor::RESET : "";
            // Use cerr for error-class tags so they surface even when stdout is
            // redirected, otherwise use cout.
            if (tag == "ERROR") {
                fprintf(stderr, "%s[%s][%d] %s%s\n", color, tag_buf, debug_level, message.c_str(), reset);
            } else {
                fprintf(stdout, "%s[%s][%d] %s%s\n", color, tag_buf, debug_level, message.c_str(), reset);
            }
        }
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

    static void logRS(const std::string& message) {
        log("REALSENSE", message, 0);
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
        log("SAVE", message, 1);
    }

    static void logTurntable(const std::string& message) {
        log("TURNTABLE", message, 1);
    }

    // always_log = true — original logInfo/logWarning had no config guard
    static void logInfo(const std::string& message) {
        log("INFO", message, 0, /*always_log=*/true);
    }

    static void logWarning(const std::string& message) {
        log("WARNING", message, 0, /*always_log=*/true);
    }

    static void logSerial(const std::string& message) {
        log("SERIAL", message, 0);
    }

    static void logWhitespace(int n = 1) {
    std::lock_guard<std::mutex> lock(log_mutex);
    for (int i = 0; i < n; i++) {
        fprintf(stdout, "\n");
    }
}


    // -------------------------------------------------------------------------
    // OLD: DONT USE THESE IN NEW CODE
    // Print-only wrappers (terminal output, no file write)
    // Preserved from original; these check the "debug" config flag, not
    // "log_debug", so they are kept separate from log() rather than folded in.
    // -------------------------------------------------------------------------

    // static void printDebug(const std::string& message) {
    //     if (!_debugEnabled()) return;
    //     std::cout << "[DEBUG] " << message << std::endl;
    // }

    // static void printError(const std::string& message) {
    //     if (!_debugEnabled()) return;
    //     std::cerr << "[ERROR] " << message << std::endl;
    // }

    // static void printInfo(const std::string& message) {
    //     if (!_debugEnabled()) return;
    //     std::cout << "[INFO] " << message << std::endl;
    // }

    // static void printWarning(const std::string& message) {
    //     if (!_debugEnabled()) return;
    //     std::cout << "[WARNING] " << message << std::endl;
    // }


    // -------------------------------------------------------------------------
    // Timer helpers
    // -------------------------------------------------------------------------

    static void startTimer() {
        if (!_debugEnabled()) return;
        start_time = std::chrono::high_resolution_clock::now();
    }

    static void stopTimer(const std::string& message) {
        if (!_debugEnabled()) return;
        end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "[TIMER] " << message << " took " << duration.count() << " ms" << std::endl;
    }


private:

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    static bool _debugEnabled() {
        return ConfigHandler::getInstance().getValue<bool>("debug");
    }
};