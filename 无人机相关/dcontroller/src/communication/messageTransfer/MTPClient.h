#pragma once
#include <string>
#include <asio.hpp>
#include "DatagramProcessor.h"

using std::string;
using namespace asio::ip;

// Make a connection call to the server, and forward the received datagrams to a
// DatagramProcessor.
class MTPClient{
public:
    MTPClient(asio::io_context& ioContext);

    void registerMessageProcessor( function<
                                   void( MessageType messageType,
                                         const string& message) > processor);

    void connect(const string& serverAddress, const string& serverPort);

private:
    void startRead();

private:
    asio::io_context& ioContext;

    // endpoints obtained by resolving server.
    udp::endpoint serverEndpoint;

    // The UDP socket used for communication with server.
    udp::socket socket_;

    // Each datagram received from the server will be saved into this buffer first,
    // then the buffer is passed to DatagramProcessor for further processing.
    char datagram[DatagramMaxLength];

    // Each received datagram will be forwarded to the following object.
    DatagramProcessor datagramProcessor;
};
