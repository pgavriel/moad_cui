#include "MenuHandler.h"

using namespace std::chrono_literals;

MenuHandler::MenuHandler(
    std::map<std::string, std::string> menu,
    std::map<std::string, bool (*)()> actions)
    : 
    menu_list(menu), menu_action(actions)
{
    this->title = "";
    this->help_title = "('r' to return)";
}

void MenuHandler::initialize() {
    std::string input;
    do {
        // Show Menu
        this->ShowMenu();
        // Get input
        std::cout << "> ";
        std::cin >> input;
        // Execute function
        if (input != "r" && input != "R") {
            this->SelectMenu(input);
        } 
        else {
            std::cout << "Returning..." << std::endl;
        }
        // Clean Screen
        this->ClearScreen();
        std::this_thread::sleep_for(100ms);
    }
    while (input != "r" && input != "R");
}

void MenuHandler::ShowMenu() {
    tabulate::Table table;
    for (const auto& pair: menu_list) {
        table.add_row({pair.first, pair.second});
    }

    table.format().border("").border_top("").corner("");
    table.column(0).format().width(3);
    if (this->title != "") {
        std::cout << this->title << std::endl;
    }
    if (this->help_title != "") {
        std::cout << this->help_title << std::endl;
    }
    std::cout << table << std::endl;
}

void MenuHandler::SelectMenu(std::string item){
    // Check if option exists
    if (menu_action.find(item) == menu_action.end() || menu_list.find(item) == menu_list.end()){
        std::cout << "Option [" << item << "] is invalid";
    }
    else {
        // Clear Screenvoid
        this->ClearScreen();
        // Display Menu Title
        std::cout << menu_list[item] << std::endl;
        // Execute Menu Action
        bool is_submenu = menu_action[item]();
        // Wait until Key Press
        if (!is_submenu)
            this->WaitUntilKeypress();
    }
}

void MenuHandler::ClearScreen() {
    std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
}

void MenuHandler::WaitUntilKeypress() {
    std::string dummy_input;
    std::cout << "Press any key + Enter to continue ..." << std::endl;
    std::cin >> dummy_input; 
}

void MenuHandler::setTitle(std::string title) {
    this->title = title;
}
void MenuHandler::setHelpTitle(std::string help_title) {
    this->help_title = help_title;
}