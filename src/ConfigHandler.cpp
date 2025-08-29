#include <iostream>
#include <fstream>

#include <ConfigHandler.h>

ConfigHandler::ConfigHandler() {
    // Constructor
    std::cout << "Instance Created: ConfigHandler" << std::endl;
}

ConfigHandler::~ConfigHandler() {
    // Destructor
}

void ConfigHandler::loadConfig(const std::string& filepath) {
    std::ifstream json_file(filepath);
	config = nlohmann::json::parse(json_file);
	json_file.close();
}

void ConfigHandler::saveConfig(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (file.is_open()) {
        file << config.dump(4); // Pretty print with 4 spaces indentation
        file.close();
        std::cout << "Camera configuration saved to " << filepath << std::endl;
    } else {
        std::cerr << "Failed to open file for writing: " << filepath << std::endl;
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