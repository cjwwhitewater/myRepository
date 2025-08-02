#pragma once
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/circular_buffer.hpp>
#include <glib.h>
#include <gst/rtsp-server/rtsp-server.h>

class RTSPPresenter{
public:
    RTSPPresenter();
    ~RTSPPresenter();

    // Start the RTSP server.
    void startRTSPServer();

    // Once one packet of H264 packet has been generated, this
    // function will be called.
    void publishVideoData(const uint8_t* buf, uint32_t len);

    GstFlowReturn onNewSample(GstElement* appsrc, int size);

private:
    void RTSPServingThreadFunction();

private:
    std::shared_ptr<std::thread> rtspServingThread;
    GstRTSPMediaFactory* factory;

    boost::circular_buffer<char> circularBuffer;
    std::mutex circularBufferMutex;
    std::condition_variable conditionalVariable;
};
