#pragma once
#include <cstdint>
#include <string>
#include <map>

// When the client splits a message into multiple UDP datagrams, each datagram must have
// a length less or equal to the following limit.
const int DatagramMaxLength = 10'000;

enum class MessageType{
    DeviceStatus,
    GridData,        // Related to the instruction checking. It is a json-format string,
                     // the data object is a matrix, whose (i,j)-th element represents
                     // the cell status of the grid. Possible values of the status:
                     // 'F' --> free
                     // 'R' --> required
                     // 'O' --> obstacle
                     // 'C' --> conflict
    Image,
    Video
};

extern std::map<MessageType, std::string> messageTypeNames;

// Header of each datagram.
struct DatagramHeader {
    uint32_t messageType;
    uint32_t messageID;
    uint32_t messageLength;  // in bytes
    uint32_t datagramID;
}__attribute__((packed));
