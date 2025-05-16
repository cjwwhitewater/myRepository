#pragma once
#include <jsoncpp/json/json.h>

// Process the instruction received from the instruction machine.
void processPTZControlInstruction(const Json::Value& instruction);
