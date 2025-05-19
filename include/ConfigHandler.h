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
                std::cerr << "Key not found: " << k << " in " << key << std::endl;
                throw std::runtime_error("Key not found: " + k + " in " + key);
            }
        }

        return current->get<T>();
    };
    
    template <typename T>
    void setValue(const std::string& key, const T& value) {
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
    };


private:
    nlohmann::json config;

    ConfigHandler();
    ~ConfigHandler();
    std::vector<std::string> split(std::string str, char delimiter);
};