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

#include "ConfigHandler.h"

class DebugUtils {
private:
    DebugUtils() {}
    ~DebugUtils() {}
    DebugUtils(const DebugUtils&) = delete;
    DebugUtils& operator=(const DebugUtils&) = delete;

    static std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    static std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    static std::ofstream log_file;

public:

    // -------------------------------------------------------------------------
    // Log file lifecycle
    // -------------------------------------------------------------------------

    static void initLogFile(const std::string& file_path) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }

        log_file.open(file_path, std::ios::out | std::ios::app);
        if (!log_file.is_open()) {
            std::cerr << "[ERROR] Failed to open log file: " << file_path << std::endl;
        } else {
            std::cout << "Opened Debug Log File: " << file_path << std::endl;
        }
    }

    static void closeLogFile() {
        if (log_file.is_open()) {
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
    // -------------------------------------------------------------------------

    static void log(const std::string& tag,
                    const std::string& message,
                    int  debug_level = 0,
                    bool always_log  = false)
    {
        ConfigHandler& config = ConfigHandler::getInstance();

        const bool file_enabled = always_log || config.getValue<bool>("log_debug");
        if (file_enabled && log_file.is_open()) {
            log_file << "[" << tag << "] " << message << std::endl;
        }

        const int verbosity = config.getValue<int>("debug_verbosity");
        if (verbosity >= debug_level) {
            // Use cerr for error-class tags so they surface even when stdout is
            // redirected, otherwise use cout.
            if (tag == "ERROR") {
                std::cerr << "[" << tag << "] " << message << std::endl;
            } else {
                std::cout << "[" << tag << "] " << message << std::endl;
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


    // -------------------------------------------------------------------------
    // Print-only wrappers (terminal output, no file write)
    // Preserved from original; these check the "debug" config flag, not
    // "log_debug", so they are kept separate from log() rather than folded in.
    // -------------------------------------------------------------------------

    static void printDebug(const std::string& message) {
        if (!_debugEnabled()) return;
        std::cout << "[DEBUG] " << message << std::endl;
    }

    static void printError(const std::string& message) {
        if (!_debugEnabled()) return;
        std::cerr << "[ERROR] " << message << std::endl;
    }

    static void printInfo(const std::string& message) {
        if (!_debugEnabled()) return;
        std::cout << "[INFO] " << message << std::endl;
    }

    static void printWarning(const std::string& message) {
        if (!_debugEnabled()) return;
        std::cout << "[WARNING] " << message << std::endl;
    }


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