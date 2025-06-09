#pragma once
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include "JoystickCommand.h"

using std::unique_ptr;
using std::mutex;
using std::condition_variable;

class InstantCommandExecuter{
public:
    void setCommand(const JoystickCommand& command_);

private:
    // The thread function to continuously send commands to ugv.
    void commandSendingThreadFunction();

    // Execute the new command.
    void executeCommand();

private:    
    // thread control
    unique_ptr<std::thread> commandSendingThread;

    // This mutex to control the concurrent access of the following variables.
    mutex sharedDataMutex;        
    bool toTerminateThread;
    JoystickCommand command;
    // If there is no new command, the thread is blocked. When a new command
    // has arrived, the thread will be awaken. The above effect is acheived by
    // using the following members.
    bool newCommandAvailable;
    condition_variable cvForArrivalOfNewCommand;

// runtime singleton
public:
    static InstantCommandExecuter* createInstance();
    static InstantCommandExecuter* get();
private:
    InstantCommandExecuter();
    ~InstantCommandExecuter();

private:
    static InstantCommandExecuter* singleton;
};
