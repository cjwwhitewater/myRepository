#include <iostream>
#include <set>
#include <string>
#include <asio.hpp>
#include <asio/deadline_timer.hpp>
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
#include "ptzControlling/GimbalDriver.h"
#include "ptzControlling/FocalLengthController.h"
#include "videoProcessing/videoReading/VideoReader.h"
#include "videoProcessing/rtspPresenting/RTSPPresenter.h"
#include "safetyChecking/SafetyChecker.h"
#include "droneControlling/DroneStatusQuerier.h"

using namespace std;

// This function distributes instructions received by the vcontroller.
void setupETTServer(ETTServer& server)
{
    server.registerConnectionProcessor(
                [](const string& remoteIP, shared_ptr<Packager> packager){
        cout << "established an ETT connection for remove IP:"
             << remoteIP << endl;

        // Let the SafetyChecker to record this pointer.
        SafetyChecker::get()->setETTPackager(packager);

        packager->registerTextProcessor( [packager](const string& text){
            options::Options& ops = options::OptionsInstance::get();
            // parse the text.
            Json::Reader reader;
            Json::Value instructionData;
            reader.parse(text, instructionData);

            // heart beat processing
            if (instructionData["essentialTextType"].asString() == "heatbeat"){
                HeartBeatProcessor* heartBeatProcessor =
                        HeartBeatProcessor::get();
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

            // process gimbal control and zoom instructions
            if (ops.presents("cameraPresents")){
                if ( instructionID == "D16"){   // camera focal length
                    int focalLength =
                            instructionData["instructionParameter"].asInt();
                    FocalLengthController::get()->setFocalLength(focalLength);
                    return;
                }
                if ( instructionID == "D17"){   // gimbal pitch control
                    int pitch = instructionData["instructionParameter"].asInt();
                    GimbalDriver::get()->setGimbalPitch(pitch);
                    return;
                }
                if ( instructionID == "D18"){   // gimbal yaw control
                    int yaw = instructionData["instructionParameter"].asInt();
                    GimbalDriver::get()->setGimbalYaw(yaw);
                    return;
                }
            }

            // Fan and light controlling.
            if ( instructionID == "D27"){
                FanController *fanController = FanController::get();
                int gear = instructionData["instructionParameter"].asInt();
                fanController->setGear(gear);
                return;
            }
            if ( instructionID == "D28"){
                LightController *lightController = LightController::get();
                int brightness = instructionData["instructionParameter"].asInt();
                lightController->setBrightness(brightness);
                return;
            }

            // Other instructions are related to drone control.
            ControlModeManager* controlModeManager = ControlModeManager::get();
            if (controlModeManager->canProcessInstruction(instructionID)){
                controlModeManager->processInstruction(instructionData);
            }

            // Instruction D19 and D20 are processed by the ibox directly.

        });
        packager->startRead();
    });

    server.start();
}

// This function will be periodically called. Each time it is called, it informs
// the current controller to determine a JoystickCommand, and send the command
// to the InstantCommandExecuter.
void performDroneControlling(const asio::error_code& /*ec*/,
                             asio::deadline_timer* timer,
                             boost::posix_time::milliseconds* interval)
{
    HeartBeatProcessor* heartBeatProcessor = HeartBeatProcessor::get();
    if ( heartBeatProcessor->lastHeartBeatIsFresh()){
        ControlModeManager::get()->executeTimerTask();
    }

    // repeat this timer
    timer->expires_at(timer->expires_at() + *interval);
    timer->async_wait(std::bind(performDroneControlling,
                                std::placeholders::_1, timer,
                                interval) );
}

// This function reports status of drone and H30 camera.
void reportDroneSideStatus(const asio::error_code& /*e*/,
                           asio::deadline_timer* timer,
                           boost::posix_time::milliseconds* interval,
                           MTPServer* mtpServer)
{
    // Basisc device status.
    Json::Value status;
    static int statusID = 0;
    status["statusID"] = statusID++;

    // Let each device/object report its status.
    DroneStatusQuerier::get()->reportStatus(status);
    GimbalDriver::get()->reportStatus(status);
    FocalLengthController::get()->reportStatus(status);
    ControlModeManager::get()->reportStatus(status);

    // send out the device status and soldier position.
    if (mtpServer->isConnected())
        mtpServer->writeMessage( MessageType::DeviceStatus,
                                 status.toStyledString() );

    // repeat this timer
    timer->expires_at(timer->expires_at() + *interval);
    timer->async_wait(std::bind(reportDroneSideStatus,
                                std::placeholders::_1, timer,
                                interval, mtpServer) );
}

void destructSingletons()
{
    delete GimbalDriver::get();
}

void createSingletons()
{
    HeartBeatProcessor::createInstance();
    cout << "created an instance of HeartBeatProcessor" << endl;

    // simple peripheral devices of onx.
    Attacker::createInstance();
    cout << "created an instance of Attacker" << endl;

    FanController::createInstance();
    cout << "created an instance of FanController" << endl;

    LightController::createInstance();
    cout << "created an instance of LightController" << endl;

    // PTZ controlling related.
    GimbalDriver::createInstance();
    cout << "created an instance of GimbalDriver" << endl;

    FocalLengthController::createInstance();
    cout << "created an instance of FocalLengthController" << endl;

    // flight control related.
    DroneStatusQuerier::createInstance();
    cout << "created an instance of DroneStatusQuerie" << endl;

    InstantCommandExecuter::createInstance();
    cout << "created an instance of InstantCommandExecuter" << endl;

    ControlModeManager::createInstance();
    cout << "created an instance of ControlModeManager" << endl;
    ControlModeManager::get()->initialize();

    SafetyChecker::createInstance();
    cout << "created an instance of SafetyChecker" << endl;
}

int main(int argc, char* argv[])
{
    cout << "dcontroller version 2.2" << endl;
    string applicationOptionsFilename = "config/dcontroller-config.txt";
    cout << "to read config from: " << applicationOptionsFilename << "\n";
    options::OptionsInstance instance( applicationOptionsFilename );
    options::Options& ops = options::OptionsInstance::get();

    if (initializePSDK(argc, argv) != 0){
        cerr << "failed to initialize PSDK, exiting..." << endl;
        return -1;
    }

    if (gpioInitialise() < 0) {
        cerr << "JETGPIO initialization failed!" << endl;
        return -1;
    }

    createSingletons();

    RTSPPresenter rtspPresenter;
    rtspPresenter.startRTSPServer();

    // Start H30 H264 streaming and forward the stream data to RTSPPresenter.
    VideoReader videoReader;
    videoReader.setVideoDataProcessor(
                [&rtspPresenter](const uint8_t*buf, uint32_t len){
            rtspPresenter.publishVideoData(buf, len);
    });
    videoReader.startVideoStream();

    try{
        // The kernel io_context for the main event loop.
        // When the program is terminated by the Ctrl+C keyboard event,
        // the 'stop' member function of this io_context is executed to
        // terminate the event loop.
        asio::io_context ioContext;

        // Set processor for Ctrl+C signal triggered by user.
        asio::signal_set signals(ioContext, SIGINT, SIGTERM);
        signals.async_wait(
            [&](std::error_code /*ec*/, int /*signo*/) {
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

        // Setup a periodical timer to report device status
        boost::posix_time::milliseconds intervalA(200);
        asio::deadline_timer timerA(ioContext, intervalA);
        timerA.async_wait( std::bind(reportDroneSideStatus,
                                    std::placeholders::_1,
                                    &timerA, &intervalA,
                                    &mtpServer) );

        // Setup another periodical timer for performing various controlling.
        boost::posix_time::milliseconds intervalB(50);
        asio::deadline_timer timerB(ioContext, intervalB);
        timerB.async_wait( std::bind(performDroneControlling,
                                    std::placeholders::_1,
                                    &timerB, &intervalB) );

        ioContext.run();
    }catch (std::exception& e){
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
