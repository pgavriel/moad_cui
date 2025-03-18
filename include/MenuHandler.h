#include <iostream>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>

#include "tabulate.hpp"

class MenuHandler {
private:
    std::map<std::string, std::string> menu_list;
    std::map<std::string, bool (*)()> menu_action;
    std::map<std::string, std::string>& info_list;
    std::string title;
    std::string help_title;
    std::string message;
    std::mutex print_mtx;
public:
    MenuHandler(
        std::map<std::string, std::string>, 
        std::map<std::string, bool (*)()>,
        std::map<std::string, std::string>&
    ); 
    void initialize();
    void ShowMenu();
    void SelectMenu(std::string key);
    void ClearScreen();
    void WaitUntilKeypress();
    void setTitle(std::string);
    void setHelpTitle(std::string);
    void addMessage(std::string);
};
