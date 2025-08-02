#pragma once
#include <string>
#include <memory>
#include <set>
#include "magic_enum.hpp"
#include "../droneControlling/Instruction.h"
#include "../communication/messageTransfer/MTPServer.h"
#include "../communication/essentialTextTransfer/Packager.h"
#include "Perceptor.h"

using std::string;

class SafetyChecker{
public:
    void setETTPackager(shared_ptr<Packager> packager);

    // Return true if the instruction is valid (safe to execute it).
    // The instruct should fall in the pre-defined set. Otherwise,
    // this function always return true.
    bool checkInstruction(Instruction instruction, double speed);

    // Send the checking result by using the ETTServer.
    void sendCheckingResult(int essentialTextID, Instruction instructionID,
                            bool isValid);
    // Send an empty checking result to the client so that the latter can
    // erase the content of checking result widget.
    void sendEmptyCheckingResult(int essentialTextID,
                                 Instruction instruction);

private:
    // Determine a braking distance according to the instruction and specified speed.
    double determineBrakingDistance(Instruction instruction, double speed);

private:
    // Define a set which contains all instructions to be checked by this class.
    std::set<Instruction> instructions;

    Perceptor perceptor;

    // For communication with client.
    std::shared_ptr<Packager> packager;

// runtime singleton
public:
    static SafetyChecker* createInstance();
    static SafetyChecker* get();
private:
    SafetyChecker();
    ~SafetyChecker();

private:
    static SafetyChecker* singleton;
};
