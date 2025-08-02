#pragma once
#include <memory>
#include <asio.hpp>

using std::shared_ptr;
using std::string;
using namespace asio;
using namespace asio::ip;

class Packager;
class ETTClient{
public:
    ETTClient(asio::io_context& ioContext);

    // This is a blocking operation.
    void connectToServer(const string& serverAddress, const string& listeningPort);

    // This function can only be called after calling to 'connectToServer'.
    shared_ptr<Packager> getPackager();

private:
    asio::io_context& ioContext;    
    shared_ptr<Packager> packager;
};
