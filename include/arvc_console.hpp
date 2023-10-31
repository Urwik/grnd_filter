#pragma once

#include <iostream>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

class console
{
private:
    /* data */
public:
    console(/* args */);
    ~console();

    void info(const string msg);
    void warning(const string msg);
    void error(const string msg);
    void debug(const string msg);

    bool enable;
    bool enable_info;
    bool enable_warning;
    bool enable_error;
    bool enable_debug;
};

console::console(/* args */)
{
    enable = true;
    enable_info = true;
    enable_warning = true;
    enable_error = true;
    enable_debug = true;
}

console::~console()
{
}


void console::info(const string msg)
{
    if (enable_info && enable)
        std::cout << GREEN << msg << RESET << std::endl;
}

void console::warning(const string msg)
{
    if (enable_warning && enable)
        std::cout << YELLOW << msg << RESET << std::endl;
}

void console::error(const string msg)
{
    if (enable_error && enable)
        std::cout << RED << msg << RESET << std::endl;
}

void console::debug(const string msg)
{
    if (enable_debug && enable)
        std::cout << msg << std::endl;
}