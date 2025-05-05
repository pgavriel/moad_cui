#include <iostream>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>

#include "tabulate.hpp"

enum MenuMessageStatus {
    SUCCESS,
    WARNING,
    ERR,
    INFO
};

class MenuHandler {
private:
    std::map<std::string, std::string> menu_list;
    std::map<std::string, bool (*)()> menu_action;
    std::map<std::string, std::string>& info_list;
    std::string title;
    std::string help_title;
    tabulate::Table message;
    std::mutex print_mtx;
public:
    MenuHandler(
        std::map<std::string, std::string>, 
        std::map<std::string, bool (*)()>,
        std::map<std::string, std::string>&
    ); 
    void initialize(MenuHandler*&);
    void ShowMenu();
    void SelectMenu(std::string key);
    void ClearScreen();
    static void WaitUntilKeypress();
    void setTitle(std::string);
    void setHelpTitle(std::string);
    void addMessage(MenuMessageStatus, std::string);
};
