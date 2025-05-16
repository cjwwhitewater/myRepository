#include <iostream>
#include <set>
#include <jsoncpp/json/json.h>
#include <boost/date_time.hpp>
#include <jetgpio.h>
#include "options.h"
#include "communication/essentialTextTransfer/ETTServer.h"
#include "communication/essentialTextTransfer/Packager.h"
#include "communication/messageTransfer/MTPServer.h"
#include "droneControlling/ControlModeManager.h"
#include "droneControlling/HeartBeatProcessor.h"
#include "gpioManipulators/Attacker.h"
#include "gpioManipulators/FanController.h"
#include "gpioManipulators/LightController.h"
#include "PSDKInitialization/PSDKInitializer.h"
#include "cameraControlling/cameraControlling.h"
#include "cameraControlling/GimbalDriver.h"

using namespace std;

// This function distributes instructions received by the vcontroller.
void setupETTServer(ETTServer &server)
{
    server.registerConnectionProcessor(
        [](const string &remoteIP, shared_ptr<Packager> packager)
        {
            cout << "established an ETT connection for remove IP:" << remoteIP << endl;
            packager->registerTextProcessor([packager](const string &text)
                                            {
            options::Options& ops = options::OptionsInstance::get();
            // parse the text.
            Json::Reader reader;
            Json::Value instructionData;
            reader.parse(text, instructionData);

            // heart beat processing
            if (instructionData["essentialTextType"].asString() == "heatbeat"){
                HeartBeatProcessor* heartBeatProcessor = HeartBeatProcessor::get();
                heartBeatProcessor->updateHeartBeatTimeStamp();
                return;
            }

            cout << "received a text: " << text << endl;

            // First check the instruction.
            //int checkedMessageID  = instructionData["essentialTextID"].asInt();
            string instructionID = instructionData["instructionID"].asString();

            // distribute the instruction
            // simulated attacking
            if ( instructionID == "D14" || instructionID == "D15"){
                Attacker* attacker = Attacker::get();
                if (instructionID == "D14")
                    attacker->singleShot();
                else
                    attacker->multipleShots();
                return;
            }

            // process camera PTZ control instructions
            if (ops.presents("cameraPresents")){
                set<string> ptzControlInstructions={"D16", "D17", "D18"};
                if (ptzControlInstructions.count(instructionID)){
                    processPTZControlInstruction(instructionData);
                    return;
                }
            }

            // Instruction D19 and D20 are processed by the ibox directly.

            // Other instructions are related to drone control.
            ControlModeManager* controlModeManager = ControlModeManager::get();
            if (controlModeManager->canProcessInstruction(instructionID)){
                controlModeManager->processInstruction(instructionData);
            } });
            packager->startRead();
        });

    server.start();
}

//// This function will be periodically called. Each time it is called, it informations
//// the current controller to determine target speed/turning angle, and send those targets
//// to the VehicleCommander.
// void performUGVControlling(const asio::error_code& /*e*/,
//                            asio::deadline_timer* timer,
//                            boost::posix_time::milliseconds* interval)
//{
//     HeartBeatProcessor* heartBeatProcessor = HeartBeatProcessor::get();
//     if ( heartBeatProcessor->lastHeartBeatIsFresh()){
//         ControlModeManager* controlModeManager = ControlModeManager::get();
//         VehicleCommander* vehicleCommander = VehicleCommander::get();
//         double targetSpeed=0, targetAngle=0;
//         controlModeManager->determineTagetSpeedAndAngle(targetSpeed, targetAngle);
//         vehicleCommander->setSpeedAndAngle(targetSpeed, targetAngle);
//     }

//    // repeat this timer
//    timer->expires_at(timer->expires_at() + *interval);
//    timer->async_wait(std::bind(performUGVControlling,
//                                std::placeholders::_1, timer,
//                                interval) );
//}

//// This function reports status of UGV and PTZ camera.
// void reportUGVAndPTZStatus(const asio::error_code& /*e*/,
//                            asio::deadline_timer* timer,
//                            boost::posix_time::milliseconds* interval,
//                            MTPServer* mtpServer)
//{
//     // Basisc device status.
//     VehicleCommander* ugv = VehicleCommander::get();
//     ControlModeManager* controlModeManager = ControlModeManager::get();
//     Json::Value status;
//     static int statusID = 0;
//     double voltage, velocity, angle;
//     ugv->getStatus(voltage, velocity, angle);
//     status["statusID"]         = statusID++;
//     status["ugvEnabled"]       = controlModeManager->UGVEnabled()?1:0;
//     status["ugvBattery"]       = voltage;
//     status["ugvVelocity"]      = velocity;
//     status["ugvSteeringAngle"] = angle;

//    // The current UGV controller may append extra status information.
//    controlModeManager->appendControllerStatus(status);

//    // Append PTZ status information.
//    OnvifAgent* onvifAgent = OnvifAgent::get();
//    float yaw, pitch, focalLength;
//    if (onvifAgent->getUserSpacePTZ(yaw, pitch, focalLength)){
//        status["cameraYaw"]         = (int)yaw;
//        status["cameraPitch"]       = (int)pitch;
//        status["cameraFocalLength"] = (int)focalLength;
//    }

//    // send out the device status and soldier position.
//    if (mtpServer->isConnected())
//        mtpServer->writeMessage( MessageType::DeviceStatus,
//                                 status.toStyledString() );

//    // repeat this timer
//    timer->expires_at(timer->expires_at() + *interval);
//    timer->async_wait(std::bind(reportUGVAndPTZStatus,
//                                std::placeholders::_1, timer,
//                                interval, mtpServer) );
//}

//// This function will be periodically called. Each time it is called, it checks
//// any possibly occlision with environmental obstacles.
// void checkCollision(const asio::error_code& /*e*/,
//                     asio::deadline_timer* timer,
//                     boost::posix_time::milliseconds* interval)
//{
//     ControlModeManager* controlModeManager = ControlModeManager::get();
//     SafetyChecker* safetyChecker = SafetyChecker::get();
//     bool safeToDrive = safetyChecker->safeToDrive();
//     if (!safeToDrive &&
//         controlModeManager->getControlMode() != SoldierFollowingMode){
//         // stop ugv
//         ControlModeManager* controlModeManager = ControlModeManager::get();
//         controlModeManager->emergencyStop();
//     }

//    // repeat this timer
//    timer->expires_at(timer->expires_at() + *interval);
//    timer->async_wait(std::bind(checkCollision,
//                                std::placeholders::_1, timer,
//                                interval) );
//}

void destructSingletons()
{
    delete GimbalDriver::get();
}

int main(int argc, char *argv[])
{
    string applicationOptionsFilename = "config/application-config.txt";
    cout << "to read config from: " << applicationOptionsFilename << "\n";
    options::OptionsInstance instance(applicationOptionsFilename);
    options::Options &ops = options::OptionsInstance::get();

    if (initializePSDK(argc, argv) != 0)
    {
        cerr << "failed to initialize PSDK, exiting..." << endl;
        return -1;
    }

    if (gpioInitialise() < 0)
    {
        cerr << "JETGPIO initialization failed!" << endl;
        return -1;
    }

    Attacker::createInstance();
    cout << "created an instance of Attacker" << endl;

    FanController::createInstance();
    cout << "created an instance of FanController" << endl;

    LightController::createInstance();
    cout << "created an instance of LightController" << endl;

    HeartBeatProcessor::createInstance();
    cout << "created an instance of HeartBeatProcessor" << endl;

    ControlModeManager::createInstance();
    cout << "created an instance of ControlModeManager" << endl;

    ControlModeManager *manager = ControlModeManager::get();
    T_DjiReturnCode ret = manager->initialize();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Flight control initialization failed, error code: 0x%08llX", ret);
        return -1;
    }

    GimbalDriver::createInstance();
    cout << "created an instance of GimbalDriver" << endl;

    try
    {
        // The kernel io_context for the main event loop.
        // When the program is terminated by the Ctrl+C keyboard event, the 'stop'
        // member function of this io_context is executed to terminate the event loop.
        asio::io_context ioContext;

        // Set processor for Ctrl+C signal triggered by user.
        asio::signal_set signals(ioContext, SIGINT, SIGTERM);
        signals.async_wait(
            [&](std::error_code /*ec*/, int /*signo*/)
            {
                ioContext.stop();
                gpioTerminate();
                destructSingletons();
                cout << "program terminated normally\n";
            });

        // Create and setup an ETT server
        ETTServer ettServer(ioContext, ops.getInt("ETTServerPort", 12345));
        setupETTServer(ettServer);

        // Create and setup an MTP server
        MTPServer mtpServer(ioContext, ops.getInt("MTPServerPort", 20000));

        //        // Setup a periodical timer to report device status(including soldier position)
        //        boost::posix_time::milliseconds intervalA(200);
        //        asio::deadline_timer deviceStatusReportTimer(ioContext, intervalA);
        //        deviceStatusReportTimer.async_wait( std::bind(reportUGVAndPTZStatus,
        //                                    std::placeholders::_1,
        //                                    &deviceStatusReportTimer, &intervalA,
        //                                    &mtpServer) );

        //        // Setup another periodical timer for performing various controlling.
        //        boost::posix_time::milliseconds intervalB(50);
        //        asio::deadline_timer ifTimer(ioContext, intervalB);
        //        ifTimer.async_wait( std::bind(performUGVControlling,
        //                                    std::placeholders::_1,
        //                                    &ifTimer, &intervalB) );

        //        // Setup another periodical timer for collision checking
        //        boost::posix_time::milliseconds intervalC(200);
        //        asio::deadline_timer collisionTimer(ioContext, intervalC);
        //        collisionTimer.async_wait( std::bind(checkCollision,
        //                                       std::placeholders::_1,
        //                                       &collisionTimer, &intervalC) );

        ioContext.run();
    }
    catch (std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
