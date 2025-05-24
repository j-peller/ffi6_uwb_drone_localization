#include "../inc/ws_logger.hpp"
#include <cstdarg>  // für va_list, va_start, va_end
#include <cstdio>   // für vsnprintf
#include <cstring>  // für strlen


int WSLogger::send_len = 0;
char* WSLogger::send_buffer = nullptr;

WSLogger& WSLogger::get_instance(const char* server_address, uint16_t port, uint16_t id)
{
    static WSLogger* instance = nullptr;

    if (!instance) {
        if (!server_address || port == 0) {
            fprintf(stderr, "[WSLogger] Invalid server address or port\n");
            fprintf(stderr, "[WSLogger] must be initialized with parameters at first call.\n");
        }
        instance = new WSLogger(server_address, port, id);
    }

    return *instance;
}

WSLogger::WSLogger(const char* server_address, uint16_t port, uint16_t id)
{
    this->id = id;
    this->server_address = server_address;
    this->port = port;
    protocols = new lws_protocols[2];

    protocols[0].name = "ws";
    protocols[0].callback = WSLogger::callback;
    protocols[0].per_session_data_size = 0;
    protocols[0].rx_buffer_size = 1024;
    protocols[0].per_session_data_size = sizeof(WSLogger*);

    protocols[1] = { nullptr, nullptr, 0, 0 }; // null terminated array

    //lws_set_log_level(LLL_USER | LLL_ERR | LLL_WARN | LLL_NOTICE | LLL_INFO | LLL_DEBUG, nullptr); /* debug messages */
    
    /* Start a new thread which handles the whole ws communication */
    thread = std::thread([this]() {
        try {
            printf("[WSLogger] Thread gestartet\n");
            this->run();
        } catch (const std::exception& e) {
            running = false;
            state = DISCONNECTED;
            printf("[WSLogger] Thread-Exception: %s\n", e.what());
        } catch (...) {
            running = false;
            state = DISCONNECTED;
            printf("[WSLogger] Thread-Exception: unbekannter Fehler\n");
            
        }
    });
}

void WSLogger::disconnect()
{
    state = DISCONNECTED;
}

void WSLogger::run()
{
    while (running) {
        //printf("TEST\n");
        switch(state)
        {
            case CONNECTED:
            {
                while (!message_queue.empty()) {
                    queue_mutex.lock();
                    std::string msg = message_queue.front();
                    message_queue.pop();
                    queue_mutex.unlock(); 

                    // Länge des JSON-Strings
                    int size = msg.length();

                    // Alte Nachricht löschen
                    delete[] send_buffer;

                    // Speicher allokieren
                    send_buffer = new char[size];
                    send_len = size;

                    // Inhalt kopieren
                    memcpy(send_buffer, msg.c_str(), size);

                    int rc = lws_callback_on_writable(wsi);
                    lws_service(context, 10);
                    
                }
                break;
            }
            case DISCONNECTED:
            {
                bool success = connect(this->server_address, this->port, "/ws-logger");
                if(success) state = CONNECTING;
                break;
            }
            default:
            {
                lws_service(context, 10);
            }
        }
    }
}

WSLogger::~WSLogger()
{
    running = false;
    disconnect();
    if (thread.joinable()) {
        thread.join();  // wartet auf sauberen Thread-Abschluss
    }
}

int WSLogger::callback(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len)
{
    WSLogger* self = static_cast<WSLogger*>(lws_wsi_user(wsi));
    
    switch (reason) {
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            printf("[WS] Connected\n");
            if (self) {
                self->sendID();
                self->state = CONNECTED;
            }
            break;
        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            printf("[WS] Connection error\n");
            self->disconnect();
            break;
        case LWS_CALLBACK_CLIENT_CLOSED:
            printf("[WS] Closed\n");
            self->disconnect();
            break;
        case LWS_CALLBACK_CLIENT_WRITEABLE:
            printf("sending %x \n", send_len);
            if (send_buffer && send_len > 0) {
                // libwebsockets verlangt Pre- und Post-Padding
                unsigned char buf[LWS_PRE + 4096];
                if (send_len > sizeof(buf) - LWS_PRE) {
                    printf("Send buffer too large\n");
                    break;
                }
                memcpy(&buf[LWS_PRE], send_buffer, send_len);

                // lws_write: LWS_WRITE_TEXT für Textnachricht
                int n = lws_write(wsi, &buf[LWS_PRE], send_len, LWS_WRITE_TEXT);
                if (n < 0) {
                    printf("[WS] Error sending data\n");
                    return -1;
                }
                printf("Sent %x \n", send_len);

                // Nach dem Senden löschen wir den Puffer
                delete[] send_buffer;
                send_buffer = nullptr;
                send_len = 0;
            }
            printf("Error sending %x \n", send_len);
            break;
        default:
            break;
    }
    return 0;
}

void WSLogger::log(const char* message, ...)
{
    va_list args;
    

    /* calculate neccesary buffer size*/
    va_start(args, message);
    int size = vsnprintf(nullptr, 0, message, args); 
    va_end(args); 

    if (size < 0) return;

    // allocate buffer
    char* buffer = new char[size];

    // format message
    va_start(args, message);
    vsnprintf(buffer, size, message, args);
    va_end(args);

    queue("log", buffer);
    delete[] buffer;
}

void WSLogger::sendID()
{
    char hex_id[16];
    snprintf(hex_id, sizeof(hex_id), "0x%04X", id);  // z. B. "0x1A2B"
    queue("id", hex_id);
}

bool WSLogger::connect(const char* address, int port, const char* path)
{
    struct lws_context_creation_info info = {};
    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = protocols;
    info.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;


    context = lws_create_context(&info);
    if (!context) return false;

    struct lws_client_connect_info ccinfo = {};
    ccinfo.context = context;
    ccinfo.address = address;
    ccinfo.port = port;
    ccinfo.path = path;
    ccinfo.host = address;
    ccinfo.origin = address;
    ccinfo.protocol = protocols[0].name;
    ccinfo.userdata = this;
   

    this->wsi = lws_client_connect_via_info(&ccinfo);
    return this->wsi != nullptr;
}

void WSLogger::queue(const char* type, const char* message)
{
    if (message_queue.size() < MAX_QUEUE_SIZE) {
        std::string json = "{\"type\": \"" + std::string(type) + "\", \"data\": \"" + std::string(message) + "\"}";
        queue_mutex.lock();
        message_queue.push(json);
        queue_mutex.unlock();
    }
}

