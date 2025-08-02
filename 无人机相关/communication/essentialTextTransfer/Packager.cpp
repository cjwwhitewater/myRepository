#include "Packager.h"
#include "../globalThreadPool.h"

using namespace std;

Packager::Packager(tcp::socket socket)
        : socket_( std::move(socket) )
{
}

void Packager::registerTextProcessor(function<void(const string& text)> processor)
{
    textProcessor = processor;
}

void Packager::startRead()
{
    socket_.async_read_some(asio::buffer(buffer, BufferSize),
        [this](std::error_code ec, std::size_t length){
          if (!ec){
              processReadData(length);
              startRead();
          }
        });
}

void Packager::processReadData(size_t length)
{
    //cout << "content of working buffer(" << length << " bytes):";
    //for (size_t i=0; i<length; i++){
    //    cout<< buffer[i];
    //}
    //cout << endl;

    for (size_t i=0; i<length; i++){
        if (buffer[i]==TextDelimiter){
            string text(inputText.data(), inputText.size());
            threadPool.push_task(textProcessor, text);
            inputText.clear();
            continue;
        }

        // Append the new character into the 'inputText'.
        inputText.push_back(buffer[i]);
        if (inputText.size() >= MAX_INPUT_TEXT_LENGTH){
            cout << "error: the accumulated input text is too large(length ="
                 << inputText.size()  << "); in normal condition, this will not happen"
                 << endl;
            inputText.clear();
        }
    }
}

void Packager::writeText(const string& text)
{    
    // First send user package synchronously.
    asio::error_code ec;
    if( ! socket_.is_open() ){
        cout << "socket closed, no writing attempt" << endl;
        return;
    }
    write(socket_, asio::buffer(text), ec);
    if (ec){
        cout << "error occurred when wrote to socket"
             << endl;
        return;
    }

    // Then send the text delimiter.
    char buffer[1];
    buffer[0] = TextDelimiter;
    if( ! socket_.is_open() ){
        cout << "socket closed, no writing attempt" << endl;
        return;
    }

    write(socket_, asio::buffer(buffer, 1), ec);
    if (ec){
        cout << "error occurred when writed to socket"
             << endl;
        return;
    }
    // cout << "sent a text: " << text << endl;
}
