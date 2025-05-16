#pragma once
#include <functional>
#include <asio.hpp>
#include "MTPCommon.h"
#include "DatagramProcessor.h"

using std::string;
using std::function;
using namespace asio::ip;

// Wait and construct a connection with the client side. In addition, the member function
// writeMessage can be called to send out a message.
class MTPServer{
public:
    MTPServer(asio::io_context& ioContext_, short port);

    // When the server receives a connection request from the client and construct such a
    // (logical) connection, the 'processor' will be called.
    void registerConnectionProcessor( function< void(const string& senderIP) > processor);

    // whether a logical connection has be established with the client.
    bool isConnected();

    // This is a blocking operation.
    void writeMessage(MessageType messageType, const string& message);

private:
    MTPServer(MTPServer&);

    void waitConnecting();

private:
    udp::socket socket_;

    // For this application, only the server is specified with a fixed IP address,
    // all the clients(such as ibox or vtester) can have arbitrary IP addresses.
    // So there must be a way for the server to know the IP address of the client.
    // This is achieved by sending a 'connection' datagram from the client to the server.
    // The following buffer is used to store this special datagram.
    char receivingBuffer[1000];
    // Endpoint of the sender for the last received datagram.
    udp::endpoint clientEndpoint;

    function< void(const string& senderIP) > connectionProcessor;

    bool connected;

    // Each time a message has been sent, the following counter will be increased by one.
    uint32_t messageIDCounter;
};
