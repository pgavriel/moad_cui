#pragma once

#include <iostream>
#include <string>
#include <chrono>

#include "ConfigHandler.h"

class DebugUtils {
private:
    // Private constructor to prevent instantiation
    DebugUtils() {}

    // Private destructor
    ~DebugUtils() {}

    // Disable copy constructor and assignment operator
    DebugUtils(const DebugUtils&) = delete;
    DebugUtils& operator=(const DebugUtils&) = delete;

    static std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    static std::chrono::time_point<std::chrono::high_resolution_clock> end_time;


public:
    // Function to print debug messages
    static void printDebug(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("debug")) {
            return; // Skip debug messages if debug mode is off
        }
        std::cout << "[DEBUG] " << message << std::endl;
    }

    // Function to print error messages
    static void printError(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("debug")) {
            return; // Skip debug messages if debug mode is off
        }
        std::cerr << "[ERROR] " << message << std::endl;
    }

    // Function to print info messages
    static void printInfo(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("debug")) {
            return; // Skip debug messages if debug mode is off
        }
        std::cout << "[INFO] " << message << std::endl;
    }

    static void printWarning(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("debug")) {
            return; // Skip debug messages if debug mode is off
        }
        std::cout << "[WARNING] " << message << std::endl;
    }

    static void startTimer() {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("debug")) {
            return; // Skip debug messages if debug mode is off
        }
        
        start_time = std::chrono::high_resolution_clock::now();
    }

    static void stopTimer(std::string message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("debug")) {
            return; // Skip debug messages if debug mode is off
        }
        
        end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "[TIMER] " << message << " took " << duration.count() << " ms" << std::endl;
    }
};