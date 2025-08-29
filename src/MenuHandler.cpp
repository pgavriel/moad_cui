// Menu Handler: It heavily relies on the tabulate library for displaying tables in the console.
// Tabulate Documentation: https://github.com/p-ranav/tabulate

#include <limits>

#include "MenuHandler.h"

using namespace std::chrono_literals;

MenuHandler::MenuHandler(
    std::map<std::string, std::string> menu,
    std::map<std::string, bool (*)()> actions,
    std::map<std::string, std::string>& info)
    : 
    menu_list(menu), menu_action(actions), info_list(info)
{
    this->title = "";
    this->help_title = "('r' to return)";
}

void MenuHandler::initialize(MenuHandler*& active_menu) {
    std::string input;
    do {
        // Show Menu
        active_menu = this;
        this->ShowMenu();

        // Get User input
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
    }
    while (input != "r" && input != "R");
}

void MenuHandler::ShowMenu() {
    tabulate::Table table;
    tabulate::Table info;

    // Fill the table with menu items
    for (const auto& pair: menu_list) {
        table.add_row({pair.first, pair.second});
    }

    // Fill the info table with information
    for (const auto& pair: info_list) {
        info.add_row({pair.first, pair.second});
    }

    // Format the table to remove borders and set width
    table.format().border("").border_top("").corner("");
    table.column(0).format().width(3);

    // Format the info table to remove inner borders and change corner styles
    info.row(0).cell(0).format().border_bottom("").corner_bottom_left("").corner_bottom_right("").border_right(":");
    info.row(0).cell(1).format().border_bottom("").corner_bottom_left("").corner_bottom_right("").border_left(":");
    info.row(1).cell(0).format().border_top("").corner_top_left("").corner_top_right("").border_right(":");
    info.row(1).cell(1).format().border_top("").corner_top_left("").corner_top_right("").border_left(":");
    info.row(1).cell(0).format().border_bottom("").corner_bottom_left("").corner_bottom_right("").border_right(":");
    info.row(1).cell(1).format().border_bottom("").corner_bottom_left("").corner_bottom_right("").border_left(":");
    info.row(2).cell(0).format().border_top("").corner_top_left("").corner_top_right("").border_right(":");
    info.row(2).cell(1).format().border_top("").corner_top_left("").corner_top_right("").border_left(":");

    // Add a header to the info table
    if (this->title != "") {
        std::cout << this->title << std::endl;
    }
    if (this->help_title != "") {
        std::cout << this->help_title << std::endl;
    }

    // Display the tables and message if available
    std::cout << "\n" << info << "\n" << std::endl;
    std::cout << this->message << "\n" << std::endl;
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

        // Execute Menu Action
        bool is_submenu = menu_action[item]();

        // Wait until Key Press
        if (!is_submenu)
            this->WaitUntilKeypress();
    }
}

// Clears the console screen by printing multiple new lines
// TODO: Implement a more robust solution for clearing the screen
void MenuHandler::ClearScreen() {
    std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
}

// Waits for the user to press Enter before continuing
void MenuHandler::WaitUntilKeypress() {
    std::cout << "Press Enter to continue ..." << std::flush;
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    std::cout << std::flush;
}

void MenuHandler::setTitle(std::string title) {
    this->title = title;
}

void MenuHandler::setHelpTitle(std::string help_title) {
    this->help_title = help_title;
}

void MenuHandler::addMessage(MenuMessageStatus status, std::string message) {
    tabulate::Table message_table;
    // Add the message to the table
    message_table.add_row({message});

    // Set the font color based on the status
    switch (status){
        case SUCCESS:
            message_table.format().font_color(tabulate::Color::green);
            break;

        case WARNING:
            message_table.format().font_color(tabulate::Color::yellow);
            break;

        case ERR:
            message_table.format().font_color(tabulate::Color::red);
            break;

        default:
            break;
    }
    
    // Add the message table to the main message
    this->message = message_table;
}