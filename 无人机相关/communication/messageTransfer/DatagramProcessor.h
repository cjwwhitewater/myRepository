#pragma once
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <functional>
#include "nanotimer.h"
#include "MTPCommon.h"

using std::string;
using std::pair;
using std::vector;
using std::map;
using std::function;

struct MessageReceivingStatus{
    vector<char> blockFlagArray;
    double datagramFirstArrivalTime;
    vector<char> message;
};

class DatagramProcessor{
public:
    DatagramProcessor();
    void registerMessageProcessor( function< void(MessageType messageType,
                                                  const string& message) > processor);

    // Process one datagram of a message. If it has been observed that
    // all blocks of a message have been accumulated, construct a complete message and
    // submit a task to execute the user-supplied message processor on the thread pool
    // to process the new message.
    void processDatagram(char* datagram, uint32_t datagramLength);

private:
    // Check each element of 'globalStatistics', if an element is too old(according to
    // its timestamp member 'datagramFirstArrivalTime'), erase it.
    void removeOutdatedStatistics();

private:
    // functor to process the last received message.
    function<
       void(MessageType messageType, const string& message)
    > messageProcessor;

    nanotimer timer;

    typedef uint32_t MessageID;
    typedef MessageID KeyType;
    map<KeyType, MessageReceivingStatus> globalStatistics;
};
