#include <iostream>
#include <options.h>
#include "MTPServer.h"

using namespace std;

MTPServer::MTPServer(asio::io_context& ioContext_, short port):
    socket_(ioContext_, udp::endpoint(udp::v4(), port))
{
    connected = 0;
    cout << "A MTP server is running on UDP port " << port << endl;

    waitConnecting();
}

void MTPServer::registerConnectionProcessor( function< void(const string& senderIP) > processor)
{
    connectionProcessor = processor;
}

bool MTPServer::isConnected()
{
    return connected;
}

void MTPServer::writeMessage(MessageType messageType, const string& message)
{
    if (!connected) return;
    if (message.empty()) return;

    // Determine number of datagrams.
    uint32_t maxPayloadLength = DatagramMaxLength - sizeof(DatagramHeader);
    uint32_t datagramNumber = (message.size()-1) /  maxPayloadLength + 1;

    options::Options& ops = options::OptionsInstance::get();
    if (ops.presents("showMTPDetails")){
        cout << "message divided into " << datagramNumber
             << " datagrams with a maximum length of " << DatagramMaxLength << " bytes" << endl;
    }

    // process each datagram.
    for (uint32_t datagramID = 0;  datagramID < datagramNumber; datagramID++){
        uint32_t blockOffset = datagramID * maxPayloadLength;
        uint32_t blockLength = maxPayloadLength;
        if (datagramID == datagramNumber-1){
            blockLength = message.size() - maxPayloadLength * (datagramNumber-1);
        }

        // construct a datagram.
        char buffer[DatagramMaxLength];
        DatagramHeader* header= reinterpret_cast<DatagramHeader*>(buffer);
        header->messageType   = static_cast<uint32_t>(messageType);
        header->messageID     = messageIDCounter;
        header->messageLength = message.size();
        header->datagramID    = datagramID;
        memcpy(buffer + sizeof(DatagramHeader),
               message.data() + blockOffset, blockLength);
        socket_.send_to(
            asio::buffer(buffer, sizeof(DatagramHeader) + blockLength),
            clientEndpoint);
        if (ops.presents("showMTPDetails")){
            cout << "sent block #" << datagramID << ", block offset = " << blockOffset
                 << ", datagram length = " << sizeof(DatagramHeader) + blockLength << endl;
        }

        // This version uses quite large a delay. We will use a more reasonable value for it.
        millisecond_delay(1);
    }
    messageIDCounter++;
}

void MTPServer::waitConnecting()
{
    socket_.async_receive_from(
        asio::buffer(receivingBuffer, sizeof(receivingBuffer)), clientEndpoint,
                     [this](const asio::error_code& ec, std::size_t bytesTransferred){
                        if (!ec){
                            string clientIP = clientEndpoint.address().to_string();
                            cout << "received a datagram from "
                                 << clientIP
                                 << " whose length is " << bytesTransferred << endl;
                            connected = 1;
                            if (connectionProcessor)
                                connectionProcessor(clientIP);
                        }
                        waitConnecting();
                    });
}
