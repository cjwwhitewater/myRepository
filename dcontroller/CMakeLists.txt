cmake_minimum_required(VERSION 3.1)
project(dcontroller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_SYSROOT /home/cjw/x-tools/aarch64-onx-linux-gnu/aarch64-onx-linux-gnu/sysroot)
# 添加 Boost 支持
set(BOOST_ROOT ${CMAKE_SYSROOT})
set(BOOST_INCLUDEDIR ${CMAKE_SYSROOT}/usr/include)
set(BOOST_LIBRARYDIR ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu)
find_package(Boost REQUIRED COMPONENTS date_time system)
include(/home/cjw/work/myRepository/packagesForJetson.cmake)
file(GLOB_RECURSE SOURCES
    src/*.cpp
)
add_executable(dcontroller ${SOURCES})
# add_executable(dcontroller src/main.cpp
#     src/droneControlling/ControlModeManager.cpp src/droneControlling/ControlModeManager.h src/droneControlling/HeartBeatProcessor.cpp src/droneControlling/HeartBeatProcessor.h
#     src/gpioManipulators/Attacker.cpp src/gpioManipulators/Attacker.h src/gpioManipulators/FanController.cpp src/gpioManipulators/FanController.h src/gpioManipulators/LightController.cpp src/gpioManipulators/LightController.h
#     src/communication/globalThreadPool.cpp src/communication/globalThreadPool.h
#     src/communication/essentialTextTransfer/ETTClient.cpp src/communication/essentialTextTransfer/ETTClient.h src/communication/essentialTextTransfer/ETTServer.cpp src/communication/essentialTextTransfer/ETTServer.h src/communication/essentialTextTransfer/Packager.cpp src/communication/essentialTextTransfer/Packager.h
#     src/communication/messageTransfer/DatagramProcessor.cpp src/communication/messageTransfer/DatagramProcessor.h src/communication/messageTransfer/MTPClient.cpp src/communication/messageTransfer/MTPClient.h src/communication/messageTransfer/MTPCommon.cpp src/communication/messageTransfer/MTPCommon.h src/communication/messageTransfer/MTPServer.cpp src/communication/messageTransfer/MTPServer.h
#     src/PSDKInitialization/PSDKInitializer.cpp src/PSDKInitialization/PSDKInitializer.h
#     src/PSDKInitialization/application/dji_sdk_app_info.h src/PSDKInitialization/application/dji_sdk_config.h
#     src/PSDKInitialization/camera_emu/dji_media_file_manage/dji_media_file_core.cpp src/PSDKInitialization/camera_emu/dji_media_file_manage/dji_media_file_core.h src/PSDKInitialization/camera_emu/dji_media_file_manage/dji_media_file_jpg.cpp src/PSDKInitialization/camera_emu/dji_media_file_manage/dji_media_file_jpg.h src/PSDKInitialization/camera_emu/dji_media_file_manage/dji_media_file_mp4.cpp src/PSDKInitialization/camera_emu/dji_media_file_manage/dji_media_file_mp4.h
#     src/PSDKInitialization/camera_emu/test_payload_cam_emu_base.cpp src/PSDKInitialization/camera_emu/test_payload_cam_emu_base.h src/PSDKInitialization/camera_emu/test_payload_cam_emu_media.cpp src/PSDKInitialization/camera_emu/test_payload_cam_emu_media.h
#     src/PSDKInitialization/camera_manager/test_camera_manager.cpp src/PSDKInitialization/camera_manager/test_camera_manager.h
#     src/PSDKInitialization/common/monitor/sys_monitor.cpp src/PSDKInitialization/common/monitor/sys_monitor.h
#     src/PSDKInitialization/common/3rdparty/FindFFMPEG.cmake src/PSDKInitialization/common/3rdparty/FindLIBUSB.cmake src/PSDKInitialization/common/3rdparty/FindOPUS.cmake src/PSDKInitialization/common/monitor/sys_monitor.cpp src/PSDKInitialization/common/monitor/sys_monitor.h src/PSDKInitialization/common/osal/osal.cpp src/PSDKInitialization/common/osal/osal.h src/PSDKInitialization/common/osal/osal_fs.cpp src/PSDKInitialization/common/osal/osal_fs.h src/PSDKInitialization/common/osal/osal_socket.cpp src/PSDKInitialization/common/osal/osal_socket.h src/PSDKInitialization/common/upgrade_platform_opt/upgrade_platform_opt_linux.cpp src/PSDKInitialization/common/upgrade_platform_opt/upgrade_platform_opt_linux.h src/PSDKInitialization/data_transmission/test_data_transmission.cpp src/PSDKInitialization/data_transmission/test_data_transmission.h src/PSDKInitialization/fc_subscription/test_fc_subscription.cpp src/PSDKInitialization/fc_subscription/test_fc_subscription.h src/PSDKInitialization/flight_control/test_flight_control.cpp src/PSDKInitialization/flight_control/test_flight_control.h src/PSDKInitialization/gimbal_emu/test_payload_gimbal_emu.cpp src/PSDKInitialization/gimbal_emu/test_payload_gimbal_emu.h src/PSDKInitialization/gimbal_manager/test_gimbal_manager.cpp src/PSDKInitialization/gimbal_manager/test_gimbal_manager.h src/PSDKInitialization/hal/hal_i2c.cpp src/PSDKInitialization/hal/hal_i2c.h src/PSDKInitialization/hal/hal_network.cpp src/PSDKInitialization/hal/hal_network.h src/PSDKInitialization/hal/hal_uart.cpp src/PSDKInitialization/hal/hal_uart.h src/PSDKInitialization/hal/hal_usb_bulk.cpp src/PSDKInitialization/hal/hal_usb_bulk.h src/PSDKInitialization/hms/data/hms.json src/PSDKInitialization/hms/hms_text/cn/hms_text_config.json src/PSDKInitialization/hms/hms_text/en/hms_text_config.json src/PSDKInitialization/hms/hms_text_c/en/hms_text_config_json.h src/PSDKInitialization/hms/test_hms.cpp src/PSDKInitialization/hms/test_hms.h src/PSDKInitialization/liveview/test_liveview.cpp src/PSDKInitialization/liveview/test_liveview.h src/PSDKInitialization/mop_channel/mop_channel_test_file/mop_send_test_file.mp4 src/PSDKInitialization/mop_channel/test_mop_channel.cpp src/PSDKInitialization/mop_channel/test_mop_channel.h src/PSDKInitialization/payload_collaboration/test_payload_collaboration.cpp src/PSDKInitialization/payload_collaboration/test_payload_collaboration.h src/PSDKInitialization/perception/test_perception.cpp src/PSDKInitialization/perception/test_perception.h src/PSDKInitialization/power_management/test_power_management.cpp src/PSDKInitialization/power_management/test_power_management.h src/PSDKInitialization/upgrade/test_upgrade.cpp src/PSDKInitialization/upgrade/test_upgrade.h src/PSDKInitialization/upgrade/test_upgrade_common_file_transfer.cpp src/PSDKInitialization/upgrade/test_upgrade_common_file_transfer.h src/PSDKInitialization/upgrade/test_upgrade_platform_opt.cpp src/PSDKInitialization/upgrade/test_upgrade_platform_opt.h src/PSDKInitialization/utils/cJSON.cpp src/PSDKInitialization/utils/cJSON.h src/PSDKInitialization/utils/dji_config_manager.cpp src/PSDKInitialization/utils/dji_config_manager.h src/PSDKInitialization/utils/util_buffer.cpp src/PSDKInitialization/utils/util_buffer.h src/PSDKInitialization/utils/util_file.cpp src/PSDKInitialization/utils/util_file.h src/PSDKInitialization/utils/util_link_list.cpp src/PSDKInitialization/utils/util_link_list.h src/PSDKInitialization/utils/util_md5.cpp src/PSDKInitialization/utils/util_md5.h src/PSDKInitialization/utils/util_misc.cpp src/PSDKInitialization/utils/util_misc.h src/PSDKInitialization/utils/util_time.cpp src/PSDKInitialization/utils/util_time.h src/PSDKInitialization/waypoint_v2/test_waypoint_v2.cpp src/PSDKInitialization/waypoint_v2/test_waypoint_v2.h src/PSDKInitialization/waypoint_v3/waypoint_file/waypoint_v3_test_file.kmz src/PSDKInitialization/waypoint_v3/waypoint_file_c/waypoint_v3_test_file_kmz.h src/PSDKInitialization/waypoint_v3/test_waypoint_v3.cpp src/PSDKInitialization/waypoint_v3/test_waypoint_v3.h src/PSDKInitialization/widget/widget_file/cn_big_screen/icon_button1.png src/PSDKInitialization/widget/widget_file/cn_big_screen/icon_button2.png src/PSDKInitialization/widget/widget_file/cn_big_screen/icon_list_item1.png src/PSDKInitialization/widget/widget_file/cn_big_screen/icon_list_item2.png src/PSDKInitialization/widget/widget_file/cn_big_screen/icon_scale.png src/PSDKInitialization/widget/widget_file/cn_big_screen/icon_switch_select.png src/PSDKInitialization/widget/widget_file/cn_big_screen/icon_switch_unselect.png src/PSDKInitialization/widget/widget_file/cn_big_screen/widget_config.json src/PSDKInitialization/widget/widget_file/en_big_screen/icon_button1.png src/PSDKInitialization/widget/widget_file/en_big_screen/icon_button2.png src/PSDKInitialization/widget/widget_file/en_big_screen/icon_list_item1.png src/PSDKInitialization/widget/widget_file/en_big_screen/icon_list_item2.png src/PSDKInitialization/widget/widget_file/en_big_screen/icon_scale.png src/PSDKInitialization/widget/widget_file/en_big_screen/icon_switch_select.png src/PSDKInitialization/widget/widget_file/en_big_screen/icon_switch_unselect.png src/PSDKInitialization/widget/widget_file/en_big_screen/widget_config.json src/PSDKInitialization/widget/widget_file_c/en_big_screen/icon_button1_png.h src/PSDKInitialization/widget/widget_file_c/en_big_screen/icon_button2_png.h src/PSDKInitialization/widget/widget_file_c/en_big_screen/icon_list_item1_png.h src/PSDKInitialization/widget/widget_file_c/en_big_screen/icon_list_item2_png.h src/PSDKInitialization/widget/widget_file_c/en_big_screen/icon_scale_png.h src/PSDKInitialization/widget/widget_file_c/en_big_screen/icon_switch_select_png.h src/PSDKInitialization/widget/widget_file_c/en_big_screen/icon_switch_unselect_png.h src/PSDKInitialization/widget/widget_file_c/en_big_screen/widget_config_json.h src/PSDKInitialization/widget/file_binary_array_list_en.cpp src/PSDKInitialization/widget/file_binary_array_list_en.h src/PSDKInitialization/widget/test_widget.cpp src/PSDKInitialization/widget/test_widget.h src/PSDKInitialization/widget/test_widget_speaker.cpp src/PSDKInitialization/widget/test_widget_speaker.h src/PSDKInitialization/widget_interaction_test/widget_file/cn_big_screen/icon_button1.png src/PSDKInitialization/widget_interaction_test/widget_file/cn_big_screen/icon_button2.png src/PSDKInitialization/widget_interaction_test/widget_file/cn_big_screen/icon_list_item1.png src/PSDKInitialization/widget_interaction_test/widget_file/cn_big_screen/icon_list_item2.png src/PSDKInitialization/widget_interaction_test/widget_file/cn_big_screen/icon_scale.png src/PSDKInitialization/widget_interaction_test/widget_file/cn_big_screen/icon_switch_select.png src/PSDKInitialization/widget_interaction_test/widget_file/cn_big_screen/icon_switch_unselect.png src/PSDKInitialization/widget_interaction_test/widget_file/cn_big_screen/widget_config.json src/PSDKInitialization/widget_interaction_test/widget_file/en_big_screen/icon_button1.png src/PSDKInitialization/widget_interaction_test/widget_file/en_big_screen/icon_button2.png src/PSDKInitialization/widget_interaction_test/widget_file/en_big_screen/icon_list_item1.png src/PSDKInitialization/widget_interaction_test/widget_file/en_big_screen/icon_list_item2.png src/PSDKInitialization/widget_interaction_test/widget_file/en_big_screen/icon_scale.png src/PSDKInitialization/widget_interaction_test/widget_file/en_big_screen/icon_switch_select.png src/PSDKInitialization/widget_interaction_test/widget_file/en_big_screen/icon_switch_unselect.png src/PSDKInitialization/widget_interaction_test/widget_file/en_big_screen/widget_config.json src/PSDKInitialization/widget_interaction_test/widget_file_c/en_big_screen/icon_button1_png.h src/PSDKInitialization/widget_interaction_test/widget_file_c/en_big_screen/icon_button2_png.h src/PSDKInitialization/widget_interaction_test/widget_file_c/en_big_screen/icon_list_item1_png.h src/PSDKInitialization/widget_interaction_test/widget_file_c/en_big_screen/icon_list_item2_png.h src/PSDKInitialization/widget_interaction_test/widget_file_c/en_big_screen/icon_scale_png.h src/PSDKInitialization/widget_interaction_test/widget_file_c/en_big_screen/icon_switch_select_png.h src/PSDKInitialization/widget_interaction_test/widget_file_c/en_big_screen/icon_switch_unselect_png.h src/PSDKInitialization/widget_interaction_test/widget_file_c/en_big_screen/widget_config_json.h src/PSDKInitialization/widget_interaction_test/file_binary_array_list_en.cpp src/PSDKInitialization/widget_interaction_test/file_binary_array_list_en.h src/PSDKInitialization/widget_interaction_test/test_widget_interaction.cpp src/PSDKInitialization/widget_interaction_test/test_widget_interaction.h src/PSDKInitialization/xport/test_payload_xport.cpp src/PSDKInitialization/xport/test_payload_xport.h
#     src/droneControlling/AccelerationController.cpp src/droneControlling/AccelerationController.h src/droneControlling/CommandExecuterBase.cpp src/droneControlling/CommandExecuterBase.h src/droneControlling/ControlModeManager.cpp src/droneControlling/ControlModeManager.h src/droneControlling/DronePosition.cpp src/droneControlling/DronePosition.h src/droneControlling/DroneStatusQuerier.cpp src/droneControlling/DroneStatusQuerier.h src/droneControlling/FlighModule.cpp src/droneControlling/FlighModule.h src/droneControlling/HeartBeatProcessor.cpp src/droneControlling/HeartBeatProcessor.h src/droneControlling/InstantCommandExecuter.cpp src/droneControlling/InstantCommandExecuter.h src/droneControlling/InstantController.cpp src/droneControlling/InstantController.h src/droneControlling/Instruction.h src/droneControlling/InstructionFollowingController.cpp src/droneControlling/InstructionFollowingController.h src/droneControlling/JoystickCommand.cpp src/droneControlling/JoystickCommand.h src/droneControlling/PersistentCommandExecuter.cpp src/droneControlling/PersistentCommandExecuter.h src/droneControlling/RTHController.cpp src/droneControlling/RTHController.h src/droneControlling/WaypointMissionController.cpp src/droneControlling/WaypointMissionController.h
#     src/ptzControlling/FocalLengthController.cpp src/ptzControlling/FocalLengthController.h src/ptzControlling/GimbalDriver.cpp src/ptzControlling/GimbalDriver.h
#     src/videoProcessing/rtspPresenting/RTSPPresenter.cpp src/videoProcessing/rtspPresenting/RTSPPresenter.h
#     src/videoProcessing/videoReading/VideoReader.cpp src/videoProcessing/videoReading/VideoReader.h
# )
# 添加编译定义（在 target_link_libraries 之前）
target_compile_definitions(dcontroller PRIVATE
    SYSTEM_ARCH_LINUX
    ASIO_ENABLE_BOOST
    ASIO_ENABLE_BOOST_DATE_TIME
)
target_link_libraries(dcontroller ${ALL_THIRD_PARTY_LIBS})
target_include_directories(dcontroller PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SYSROOT}/usr/include
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SYSROOT}/usr/local/include
    ${CMAKE_SYSROOT}/usr/include/eigen3
    ${CMAKE_SYSROOT}/usr/local/include/eigen3
    ${CMAKE_SYSROOT}/usr/include/glib-2.0
    ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/glib-2.0/include
    ${CMAKE_SYSROOT}/lib/aarch64-linux-gnu/glib-2.0/include
    ${CMAKE_SYSROOT}/usr/include/gstreamer-1.0
    ${GST_INCLUDE_DIRS}
    ${GSTRTSP_INCLUDE_DIRS}
)
target_link_libraries(dcontroller
    ${Boost_LIBRARIES}
    ${ALL_THIRD_PARTY_LIBS}
    gstreamer-1.0
    gstrtspserver-1.0
    gobject-2.0
    glib-2.0
    gstapp-1.0
    pthread
    m
)
# 添加必要的编译定义
target_compile_definitions(dcontroller PRIVATE
    SYSTEM_ARCH_LINUX
    ASIO_ENABLE_BOOST
    ASIO_ENABLE_BOOST_DATE_TIME
)
include(GNUInstallDirs)
install(TARGETS dcontroller
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
