#include <iostream>
#include <fstream>
#include <nanotimer.h>
#include <gst/app/gstappsrc.h>
#include "RTSPPresenter.h"

using namespace std;

GstFlowReturn on_new_sample(GstElement* appsrc, guint lengthToRead, gpointer user_data)
{
    //cout << "on_new_sample called, lengthToRead = " << lengthToRead << endl;
    RTSPPresenter* rtspPresenter= (RTSPPresenter*) user_data;
    return rtspPresenter->onNewSample(appsrc, lengthToRead);
}

GstFlowReturn RTSPPresenter::onNewSample(GstElement* appsrc, int lengthToRead)
{
    nanotimer timer;
    timer.start();

    unique_lock<mutex> lk(circularBufferMutex);
    conditionalVariable.wait(
        lk, [&]{return circularBuffer.size() >= lengthToRead;} );
    GstBuffer* buffer = gst_buffer_new_allocate(NULL, lengthToRead, NULL);
    char* temBuffer = new  char[lengthToRead];
    for (int i=0; i<lengthToRead; i++){
        temBuffer[i] = circularBuffer.front();
        circularBuffer.pop_front();
    }
    gst_buffer_fill(buffer, 0, temBuffer, lengthToRead);
    delete[] temBuffer;
    gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
    lk.unlock();

    //cout << "onNewSample() spent " << timer.get_elapsed_ms() << " ms" << endl;
    return GST_FLOW_OK;
}

static void media_configure(GstRTSPMediaFactory* factory,
                            GstRTSPMedia * media, gpointer user_data)
{
    // 获取管道中的appsrc元素
    GstElement* element = gst_rtsp_media_get_element(media);
    GstElement* appsrc  = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mysrc");

    // 配置appsrc
    g_object_set(appsrc,
        "stream-type", GST_APP_STREAM_TYPE_STREAM,
        "format",      GST_FORMAT_TIME,
        "is-live",     TRUE,
        NULL);

    // 设置回调函数
    g_signal_connect(appsrc, "need-data",
                     G_CALLBACK(on_new_sample),
                     user_data);

    gst_object_unref(appsrc);
    gst_object_unref(element);
}

RTSPPresenter::RTSPPresenter():circularBuffer(1'000'000)
{
}

RTSPPresenter::~RTSPPresenter()
{
    rtspServingThread->join();
}

void RTSPPresenter::startRTSPServer()
{
    rtspServingThread = make_shared<std::thread>(
                                & RTSPPresenter::RTSPServingThreadFunction, this);
}

void RTSPPresenter::RTSPServingThreadFunction()
{
    // g_setenv("GST_DEBUG", "3", TRUE);
    // g_setenv("GST_DEBUG_FILE", "gstreamer.log", 1);

    //putenv("GST_TRACERS=latency(flags=element)");
    //putenv("GST_DEBUG=GST_TRACER:7");

    gst_init(nullptr, nullptr);

    // create and setup a GstRTSPMediaFactory
    factory = gst_rtsp_media_factory_new ();
    gst_rtsp_media_factory_set_launch (factory,
         "( appsrc name=mysrc "
         "! h264parse "
         "! avdec_h264 "
         "! nvvidconv "
         "! nvv4l2h264enc "
         "  bitrate=1000000 "
         "  preset-level=1  "
         "  control-rate=1  "
         "  insert-sps-pps=1 "
         "! rtph264pay name=pay0 pt=96 mtu=1400 )" );

    // Let all clients share the same media
    gst_rtsp_media_factory_set_shared (factory, TRUE);

    // Reduce the latency
    gst_rtsp_media_factory_set_latency(factory, 0);  // 0表示自动选择最低延迟

    g_signal_connect(factory, "media-configure",
                     G_CALLBACK(media_configure), this);

    // Create the server
    GstRTSPServer* server = gst_rtsp_server_new ();

    // Set the mount point of the service
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points (server);
    gst_rtsp_mount_points_add_factory (mounts, "/stream", factory);
    g_object_unref (mounts);

    // attach the server to gst.
    if (gst_rtsp_server_attach(server, NULL) == 0) {
        g_printerr ("Failed to attach the server\n");
        return;
    }
    g_print ("RTSP server ready at rtsp://127.0.0.1:8554/stream\n");

    // Start the gst main loop.
    GMainLoop* loop = g_main_loop_new (NULL, FALSE);
    g_main_loop_run(loop);
}

void RTSPPresenter::publishVideoData(const uint8_t* data, uint32_t len)
{
    nanotimer timer;
    timer.start();
    lock_guard<mutex> lg(circularBufferMutex);
    for (int i=0; i<len; i++){
        circularBuffer.push_back(data[i]);
    }
    conditionalVariable.notify_one();
    //cout << "publishVideoData() spent " << timer.get_elapsed_ms() << " ms" << endl;
}
