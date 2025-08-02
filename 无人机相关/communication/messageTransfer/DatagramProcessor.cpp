#include <cstring>
#include <iostream>
#include "options.h"
#include "thread_pool.hpp"
#include "../globalThreadPool.h"
#include "DatagramProcessor.h"
#include "MTPCommon.h"

using namespace std;

DatagramProcessor::DatagramProcessor()
{
    timer.start();
}

void DatagramProcessor::registerMessageProcessor(
                            function< void(MessageType messageType,
                                           const string& message) > processor)
{
    messageProcessor = processor;
}

void DatagramProcessor::processDatagram(char* datagram, uint32_t datagramLength)
{
    options::Options& ops = options::OptionsInstance::get();

    DatagramHeader* header = reinterpret_cast<DatagramHeader*>(datagram);
    if ( ops.presents("showMTPDetails") ){
        cout << "reveived datagram #" << header->datagramID
             << " of message #" << header->messageID
             << " with type " << messageTypeNames.at( MessageType(header->messageType) )
             << endl;
    }

    // Determine number of datagrams.
    uint32_t maxPayloadLength = DatagramMaxLength - sizeof(DatagramHeader);
    uint32_t datagramNumber = (header->messageLength-1) /  maxPayloadLength + 1;

    // Initialize or update the receiving statistics of the mother message of this datagram
    KeyType key ((MessageID)header->messageID);
    // Statement below may insert a new element into the map if this datagram is the first
    // arrived among all of its mother message.
    vector<char>& flags   = globalStatistics[key].blockFlagArray;
    vector<char>& message = globalStatistics.at(key).message;
    if (flags.empty()){  // initializeLibAV
        // initializeLibAV the flag array of blocks.
        flags.resize(datagramNumber);
        for(auto& flag: flags){
            flag = 0;
        }

        message.resize(header->messageLength);

        globalStatistics.at(key).datagramFirstArrivalTime = timer.get_elapsed_ms();
    }
    flags.at(header->datagramID) = 1;

    // copy current block
    uint32_t blockOffset = header->datagramID * maxPayloadLength;
    memcpy(message.data() + blockOffset,
           datagram + sizeof(DatagramHeader),
           datagramLength - sizeof(DatagramHeader));

    // Determine whether all blocks of a message have been received.
    uint32_t sum = 0;
    for (auto& flag: flags){
        sum += flag;
    }
    if ( ops.presents("showMTPDetails") ){
        cout << "sum of flags: " << sum
             << " (datagramNumber=" << datagramNumber << ")" << endl;
    }
    if (sum == datagramNumber){
        // all blocks have been received.
        string messageString(message.data(), message.size());
        ::threadPool.push_task(messageProcessor,                               
                               static_cast<MessageType>(header->messageType),
                               messageString);
    }

    // Remove outdated message data. For each arrived datagram, the function below
    // will be called, but this is not a problem since the map 'globalStatistics'
    // normally contains only a few elements.
    removeOutdatedStatistics();
}

void DatagramProcessor::removeOutdatedStatistics()
{
    for (auto it = globalStatistics.cbegin(); it != globalStatistics.cend(); ) {
        if ( timer.get_elapsed_ms() - it->second.datagramFirstArrivalTime > 5000){
            it = globalStatistics.erase(it);
            continue;
        }
        it++;
    }
}
