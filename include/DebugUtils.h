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
*/



#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <fstream>

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
    static std::ofstream log_file;

public:

    // adding some functions that can write debug codes to a log file - GS 8/12

    // this init assumes the log file was created in the related codefile and the file path
    //     will be sent to this function

    static void initLogFile(const std::string& file_path) {
    // im just going to follow the same format that the previous developer used
    // with the if-statement early-return
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }

        log_file.open(file_path, std::ios::out | std::ios::app);
        if (!log_file.is_open()) {
            std::cerr << "[ERROR] Failed to open log file: " << file_path << std::endl;
        }
    }

    static void closeLogFile() {
        if (log_file.is_open()) {
            log_file.close();
        }
    }





    static void logDebug(const std::string& message) {
        // not sure if anything other than init needs the if statement
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }

        if (log_file.is_open()) {
            log_file << "[DEBUG] " << message << std::endl;
        }
    }

    static void logThread(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }

        if (log_file.is_open()) {
            log_file << "[THREAD] " << message << std::endl;
        }
    }




    static void logConfig(const std::string& message) {

        

        // ConfigHandler& config = ConfigHandler::getInstance();
        // if (!config.getValue<bool>("log_debug")) {
        //     return;
        // }

        if (log_file.is_open()) {
            log_file << "[CONFIG] " << message << std::endl;
        }
    }


    


    static void logCam(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }

        if (log_file.is_open()) {
            log_file << "[CAMERA] " << message << std::endl;
        }
    }

    static void logRS(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }

        if (log_file.is_open()) {
            log_file << "[REALSENSE] " << message << std::endl;
        }
    }






    static void logError(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }
        if (log_file.is_open()) {
            log_file << "[ERROR] " << message << std::endl;
        }
    }

    static void logEnd(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }
        if (log_file.is_open()) {
            log_file << "[END] " << message << std::endl;
        }
    }

    static void logPictureLoop(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }
        if (log_file.is_open()) {
            log_file << "[TAKE] " << message << std::endl;
        }
    }

    static void logSaveLoop(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }
        if (log_file.is_open()) {
            log_file << "[SAVE] " << message << std::endl;
        }
    }

    static void logTurntable(const std::string& message) {
        ConfigHandler& config = ConfigHandler::getInstance();
        if (!config.getValue<bool>("log_debug")) {
            return;
        }
        if (log_file.is_open()) {
            log_file << "[TURNTABLE] " << message << std::endl;
        }
    }

    static void logInfo(const std::string& message) {
        if (log_file.is_open()) {
            log_file << "[INFO] " << message << std::endl;
        }
    }

    static void logWarning(const std::string& message) {
        if (log_file.is_open()) {
            log_file << "[WARNING] " << message << std::endl;
        }
    }






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