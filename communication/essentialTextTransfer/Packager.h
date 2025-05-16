#pragma once
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "asio.hpp"
#include "thread_pool.hpp"

using std::shared_ptr;
using std::string;
using std::vector;
using std::function;
using namespace asio::ip;

class Packager: public std::enable_shared_from_this<Packager>{
public:
    Packager(tcp::socket _socket);

    // The process will be executed on the thread pool to process a received essentail text.
    void registerTextProcessor(function<void(const string& text)> processor);

    // Start an asynchronous reading operation.
    void startRead();

    // Perform a writing operation in blocking mode.
    void writeText(const string& text);

private:
    // Copy data of the current Essentail Text to the end of 'inputText'.
    // If the end mark(delimiter) of the current Text is found in the received data, call the
    // client-supplied processor to process the current Text.
    void processReadData(std::size_t length);

private:    
    tcp::socket socket_;

    const char TextDelimiter = '\0';

    // Working buffer for the asio reading function.
    enum {BufferSize = 1024};
    char buffer[BufferSize];

    // In case of exceptional condition, this class may receive extraordinary long messages
    // which do not contain the delimiter. To avoid memory overflow, we put a limit on the
    // maximum number of the input text.
    const size_t MAX_INPUT_TEXT_LENGTH = 10'000'000;   // 10MB
    vector<char> inputText;

    // Store the functor supplied by the client which will be executed when an essential text
    // has been received.
    function<void(const string& text)> textProcessor;
};
