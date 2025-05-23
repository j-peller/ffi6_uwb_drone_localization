#pragma once

#include <stdint.h>
#include <libwebsockets.h>
#include <atomic>
#include <queue>
#include <mutex>
#include <string>
#include <thread>

enum Logger_State 
{
    CONNECTED,
    CONNECTING,
    DISCONNECTED
};

class WSLogger {
    public:
        WSLogger(const char* server_address, uint16_t port, uint16_t id);
        
        /* put a message into the output queue */
        void log(const char* message, ...);
        
        ~WSLogger();
        
    private:
        /**
         * Puts a message and a type encoded in JSON format into the sending queue.
         * 
         * @param type    The type/category of the message.
         * @param message The message content as a string.
         * 
         */
        void queue(const char* type, const char* message);

        /* Wrapper for sending the current id into the queue */
        void sendID();


        static int callback(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len);
        bool connect(const char* address, int port, const char* path);
        void run();
        void disconnect();

    private:
        /* unique id for identification purposes */
        uint16_t id;

        /* web socket connection information */
        const char* server_address;
        uint16_t port;
        
        Logger_State state = DISCONNECTED;
        std::thread thread;
        std::atomic<bool> running = true;

        /* a mutex protected queue for storing logging messages */
        const size_t MAX_QUEUE_SIZE = 100;
        std::queue<std::string> message_queue;
        std::mutex queue_mutex;

        struct lws_context* context = nullptr;
        struct lws* wsi = nullptr;
        struct lws_protocols* protocols;
        static char* send_buffer;
        static int send_len;
};