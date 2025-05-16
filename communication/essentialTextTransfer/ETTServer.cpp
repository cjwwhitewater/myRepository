#include <iostream>
#include "options.h"
#include "ETTServer.h"
#include "Packager.h"

using namespace std;
using namespace options;

ETTServer::ETTServer(asio::io_context& ioContext_, int listeningPort_):
    ioContext(ioContext_), listeningPort(listeningPort_)
{
}

void ETTServer::registerConnectionProcessor(
        function<void(const string& remoteIP,
                      shared_ptr<Packager> packager)> processor)
{
    connectionProcessor = processor;
}

void ETTServer::start()
{
    Options& ops = OptionsInstance::get();
    cout << "An ETT server listens at port " << listeningPort << "\n";
    acceptor_ = make_shared<tcp::acceptor>(ioContext,
                                           tcp::endpoint(tcp::v4(), listeningPort) );

    doAccept();
}

void ETTServer::doAccept()
{
    acceptor_->async_accept([this](std::error_code ec, tcp::socket socket)
        {
          cout << "received a connection request\n";
          if (!ec){
              string remoteIP = socket.remote_endpoint().address().to_string();
              packager = make_shared<Packager>(std::move(socket));
              // Call the user supplied processor to process the new connection.
              connectionProcessor(remoteIP, packager);
          }

          // Return to the listening state.
          doAccept();
        });
}
