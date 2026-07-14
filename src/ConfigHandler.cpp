#include <iostream>
#include <fstream>

#include <ConfigHandler.h>
#include <DebugUtils.h>

ConfigHandler::ConfigHandler() {
    // Constructor
    DebugUtils::logInfo("Initializing ConfigHandler...");
}

ConfigHandler::~ConfigHandler() {
    // Destructor
}

void ConfigHandler::loadConfig(const std::string& filepath) {

    DebugUtils::logDebug("Loading Config File: " + filepath);
    // Save config path internally
    config_path = filepath;
    std::ifstream json_file(filepath);
	// config = nlohmann::json::parse(json_file);
	config = nlohmann::ordered_json::parse(json_file);
	json_file.close();
    DebugUtils::logDebug("Config loaded.");
    DebugUtils::logWhitespace();
    DebugUtils::notifyConfigReady(); // Flag to DebugUtils that a config has been loaded
}

void ConfigHandler::saveConfig(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (file.is_open()) {
        file << config.dump(4); // Pretty print with 4 spaces indentation
        file.close();

        DebugUtils::logConfig("ConfigHandler saving to: "  + filepath);

    } else {
        DebugUtils::logConfig("Failed to open file for writing: " + filepath);
	}
}

bool ConfigHandler::emptyConfig() {
    return config.empty();
}

std::vector<std::string> ConfigHandler::split(std::string str, char delimiter) {
    // Copied from: https://stackoverflow.com/questions/67486877/nlohmann-json-access-nested-value-by-single-string
    std::vector<std::string> res = {};
    std::size_t start {0};
    std::size_t end {0};
    while ((start = str.find_first_not_of(delimiter, end)) != std::string::npos) {
        end = str.find(delimiter, start);
        res.push_back(str.substr(start, end - start));
    }
    return res;
}

void ConfigHandler::writeObjectName(const std::string& object_name) {
    DebugUtils::logConfig("Updating object_name: " + object_name);
    setValue<std::string>("object_name", object_name, config_path);
}

void ConfigHandler::writePose(char pose) {
    DebugUtils::logConfig("Updating prev_state.current_pose: " + std::string(1, pose));
    setValue<int>("prev_state.current_pose", static_cast<int>(pose), config_path);
}

void ConfigHandler::writeDegreeMove(int degree, int current_move) {
    DebugUtils::logConfig("Updating prev_state.turntable_pos/current_move: "
        + std::to_string(degree) + " / " + std::to_string(current_move));
    setValue<int>("prev_state.turntable_pos", degree, config_path);
    setValue<int>("prev_state.current_move", current_move, config_path);
}