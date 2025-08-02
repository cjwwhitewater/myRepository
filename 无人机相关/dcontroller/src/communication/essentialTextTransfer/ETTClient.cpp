#include <iostream>
#include "options/options.h"
#include "ETTClient.h"
#include "Packager.h"

using namespace std;
using namespace options;

ETTClient::ETTClient(asio::io_context& ioContext_):
    ioContext(ioContext_)
{
}

void ETTClient::connectToServer(const string& serverAddress,
                                const string& listeningPort)
{
    tcp::socket s(ioContext);
    tcp::resolver resolver(ioContext);
    asio::error_code ec;
    asio::connect(s, resolver.resolve(serverAddress, listeningPort), ec);
    // report connecting result.
    if (ec){
        cout << "could not connect to "
             << serverAddress << ":" << listeningPort << endl;
        exit(-1);
    }
    cout << "connected to "
         << serverAddress << ":" << listeningPort << endl;

    packager = make_shared<Packager>(std::move(s));
}

shared_ptr<Packager> ETTClient::getPackager()
{
    return packager;
}

