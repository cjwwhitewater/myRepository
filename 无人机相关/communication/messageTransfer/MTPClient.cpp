#include <iostream>
#include "options.h"
#include "nanotimer.h"
#include "MTPCommon.h"
#include "MTPClient.h"

using namespace std;

MTPClient::MTPClient(asio::io_context& ioContext_):
    ioContext(ioContext_),
    socket_(ioContext_, udp::endpoint(udp::v4(), 0))
{
    startRead();
}

void MTPClient::connect(const string& serverAddress, const string& serverPort)
{
    // resolve server endpoint
    udp::resolver resolver(ioContext);
    udp::resolver::results_type endpoints =
            resolver.resolve(udp::v4(), serverAddress, serverPort);
    serverEndpoint = *endpoints.begin();

    // send an arbitrary datagram to the server to let it know endpoint used by client.
    string arbitraryDatagram = "connection request from client";
    socket_.send_to(asio::buffer(arbitraryDatagram), serverEndpoint);
}

void MTPClient::registerMessageProcessor( function<
                                          void(MessageType messageType,
                                               const string& message) > processor)
{
    // Just forward this message to the DatagramProcessor.
    datagramProcessor.registerMessageProcessor(processor);
}

void MTPClient::startRead()
{
    socket_.async_receive_from(
        asio::buffer(datagram, sizeof(datagram)), serverEndpoint,
                     [this](const asio::error_code& ec, std::size_t bytesTransferred){
                        if (!ec){
                            datagramProcessor.processDatagram(datagram, bytesTransferred);
                        }
                        startRead();
                    });
}
