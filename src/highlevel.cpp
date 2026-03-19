#include <string>
#include <iostream>

#include <nlohmann/json.hpp>
#include "highlevel.h"



int main(int argc, char *argv[]) {
    (void) argc;
    (void) argv;
    std::cout << "Hello World!" << std::endl;
}

HighLevelNode::HighLevelNode() : Node("high_level_node")
{
}