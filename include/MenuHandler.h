#include <iostream>
#include <string>
#include <map>

#include "tabulate.hpp"

class MenuHandler {
private:
    std::map<std::string, std::string> menu_list;
    std::map<std::string, bool (*)()> menu_action;
    std::string title;
    std::string help_title;
public:
    MenuHandler(
        std::map<std::string, std::string>, 
        std::map<std::string, bool (*)()>);
    void initialize();
    void ShowMenu();
    void SelectMenu(std::string key);
    void ClearScreen();
    void WaitUntilKeypress();
    void setTitle(std::string);
    void setHelpTitle(std::string);
};
