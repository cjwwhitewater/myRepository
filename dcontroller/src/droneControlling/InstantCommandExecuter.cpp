#include <cassert>
#include <iostream>
#include "options.h"
#include "nanotimer.h"
#include "InstantCommandExecuter.h"
#include "dji_logger.h"

using namespace std;

InstantCommandExecuter::InstantCommandExecuter()
{
    options::Options& ops = options::OptionsInstance::get();    
    toTerminateThread   = false;
    newCommandAvailable = false;
    commandSendingThread = make_unique<thread>(
                &InstantCommandExecuter::commandSendingThreadFunction, this);
}

InstantCommandExecuter::~InstantCommandExecuter()
{
    {{
    lock_guard<mutex> lg(sharedDataMutex);
    toTerminateThread = true;
    cvForArrivalOfNewCommand.notify_one();
    }}

    commandSendingThread->join();
}

void InstantCommandExecuter::commandSendingThreadFunction()
{
    while (1){
        unique_lock<mutex> lk(sharedDataMutex);
        // If there is no new command, this thread will be blocked.
        cvForArrivalOfNewCommand.wait(
            lk, [&]{return newCommandAvailable;} );
        if (toTerminateThread)
            break;

        // Execute the new command.
        executeCommand();

        newCommandAvailable = false;
        lk.unlock();
    }
}

void InstantCommandExecuter::setCommand(const JoystickCommand& command_)
{
    lock_guard<mutex> lg(sharedDataMutex);
    // set the command.
    command = command_;
    // awake the blocked thread.
    newCommandAvailable = true;
    cvForArrivalOfNewCommand.notify_one();
}

void InstantCommandExecuter::executeCommand()
{
    options::Options& ops = options::OptionsInstance::get();
    if (ops.presents("displayEachInstantCommand")){
        static int counter = 0;
        cout << "CMD#" << counter++ % 1000 << ": "
             << command.toString() << endl;
    }
    if (ops.presents("dryRun")){
        return;
    }

    DjiFlightController_SetJoystickMode(command.mode);
    millisecond_delay(50);

    T_DjiReturnCode code =
            DjiFlightController_ExecuteJoystickAction(command.command);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("execution of joystick command failed, error code:0x%08llX",
                       code);
        return;
    }
}


// Runtime singletn.
InstantCommandExecuter* InstantCommandExecuter::singleton = nullptr;
InstantCommandExecuter* InstantCommandExecuter::createInstance()
{
    assert(singleton == nullptr);
    singleton = new InstantCommandExecuter();
    return singleton;
}

InstantCommandExecuter* InstantCommandExecuter::get()
{
    assert(singleton);
    return singleton;
}
