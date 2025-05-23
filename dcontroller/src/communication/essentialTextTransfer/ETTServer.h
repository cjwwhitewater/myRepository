#pragma once
#include <functional>
#include <memory>
#include <asio.hpp>

using std::string;
using std::shared_ptr;
using std::function;
using namespace asio;
using namespace asio::ip;

class Packager;
class ETTServer{
public:
    ETTServer(asio::io_context& ioContext, int listeningPort);

    // Specify the processor when a TCP connection is established.
    void registerConnectionProcessor(function<void(const string& remoteIP,
                                     shared_ptr<Packager> packager)> processor);

    // Start listening. This is an asynchoronous operation.
    void start();

private:
    ETTServer(ETTServer&);

    void doAccept();

private:
    asio::io_context& ioContext;
    int listeningPort;
    shared_ptr<tcp::acceptor> acceptor_;

    // For one unmanned device, only one client is allowed so that there is only one
    // communication end-point can issue control command to the device. Thus, we only
    // need only one Packager on the server side to communication with the client.
    shared_ptr<Packager> packager;

    // Store the functor supplied by the client which will be executed when a TCP connection
    // is constructed.    
    function<void(const string& remoteIP,
                  shared_ptr<Packager> packager)> connectionProcessor;
};
