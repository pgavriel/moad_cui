/*
Notes: 
    This uses singleton pattern that tries to guarenteer only one confighandler instance in memory
    
    getInstance()


*/


#ifndef CONFIGHANDLER_H
#define CONFIGHANDLER_H

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

class ConfigHandler {
public:
    static ConfigHandler& getInstance() {
        static ConfigHandler instance;
        return instance;
    }

    void loadConfig(const std::string&);
    void saveConfig(const std::string&) const;
    bool emptyConfig();

    template <typename T>
    T getValue(const std::string& key) {
        // Need to deal with nested JSON objects
        // For example: config["camera"]["resolution"]
        // Split the key by '.' to access nested values
        std::vector<std::string> keys = split(key, '.');
        nlohmann::json* current = &config;
        
        for (const auto& k : keys) {
            if (current->contains(k)) {
                current = &(*current)[k];
            } else {
                // std::cerr << "Key not found: " << k << " in " << key << std::endl;
                throw std::runtime_error("Key not found: " + k + " in " + key);
            }
        }

        return current->get<T>();
    };
    
    template <typename T>
    void setValue(const std::string& key, const T& value, const std::string& config_dir) {
        // This also needs to deal with nested JSON objects
        // For example: config["camera"]["resolution"]
        // Split the key by '.' to access nested values
        std::vector<std::string> keys = split(key, '.');
        nlohmann::json* current = &config;
        
        for (const auto& k : keys) {
            if (current->contains(k)) {
                current = &current->at(k);
            } else {
                throw std::runtime_error("Key not found: " + k + " in " + key);
            }
        }

        *current = value;
        // TODO: this might save moad_config to the wrong place in some cases
        saveConfig(config_dir);
    };


private:
    nlohmann::json config;

    ConfigHandler();
    ~ConfigHandler();
    std::vector<std::string> split(std::string str, char delimiter);
};

#endif // CONFIGHANDLER_H