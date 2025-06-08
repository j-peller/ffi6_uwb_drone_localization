#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <unordered_map>
#include <nlohmann/json.hpp>

#include "extern/coords_calc/coords_calc.h" 

using json = nlohmann::json;

class DWMUartReader {
public:
    DWMUartReader(const std::string& devPath, const std::string& configPath)
        : _devPath(devPath), _running(false), _fd(-1) {
            /* Anchor mapping */
            _anchorMap = loadAnchorConfig(configPath);
        }

    ~DWMUartReader() {
        stop();
    }

    bool start() {
        _fd = open(_devPath.c_str(), O_RDWR | O_NOCTTY);
        if (_fd < 0) {
            perror("open");
            return false;
        }

        configureSerial();

        _running = true;
        _thread = std::thread(&DWMUartReader::readLoop, this);
        return true;
    }

    void stop() {
        _running = false;
        if (_thread.joinable()) _thread.join();
        if (_fd >= 0) close(_fd);
    }

    /**
     * @brief Get the latest distances to anchors from the DWM1001-DEV via UART.
     * @return distances_t containing distances to anchors with correct mapping as per config.json.
     */
    distances_t get_latest_distances_to_anchors() {
        /* idx 0 = ANCHOR_1, idx 1 = ANCHOR_2, ...*/
        distances_t result = {0.0, 0.0, 0.0, 0.0};

        /* Get latest data from UART as JSON */
        json latestData = getLatestData();

        if (latestData.empty() || !latestData.contains("rngs")) {
            std::cerr << "No valid data received." << std::endl;
            exit(EXIT_FAILURE);
        }

        /* Extract distances from the JSON object */
        auto rngs = latestData["rngs"];
        for (const auto& entry : rngs) {
            if (!entry.contains("uid") || !entry.contains("rng")) {
                std::cerr << "Invalid entry in rngs." << std::endl;
                continue;
            }

            uint32_t uid = entry["uid"];
            double value = entry["rng"].get<double>() / 1000.0; //< convert to meters

            if (_anchorMap.count(uid)) {
                size_t idx = _anchorMap[uid]; //< get the actual index from the map
                switch (idx) {
                    case 0: result.d1 = value; break;
                    case 1: result.d2 = value; break;
                    case 2: result.d3 = value; break;
                    case 3: result.d4 = value; break;
                }
            } else {
                std::cerr << "Unexpected anchor UID: 0x" << std::hex << uid << std::dec << std::endl;
            }
        }

        return result;

    }

private:
    std::unordered_map<uint16_t, size_t> loadAnchorConfig(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open anchor config file: " + filename);
        }

        json config;
        file >> config;

        std::unordered_map<std::string, size_t> labelToIndex = {
            {"d1", 0}, {"d2", 1}, {"d3", 2}, {"d4", 3}
        };

        std::unordered_map<uint16_t, size_t> uidMap;

        for (auto& [uidStr, label] : config.items()) {
            uint16_t uid = static_cast<uint16_t>(std::stoul(uidStr, nullptr, 16));
            if (labelToIndex.find(label) != labelToIndex.end()) {
                uidMap[uid] = labelToIndex[label];
            } else {
                std::cerr << "Unknown label '" << label << "' in config. Ignored." << std::endl;
            }
        }

        return uidMap;
    }


    json getLatestData() {
        std::lock_guard<std::mutex> lock(_mutex);
        return _latest;
    }

    void configureSerial() {
        struct termios tty;
        if (tcgetattr(_fd, &tty) != 0) {
            perror("tcgetattr");
            return;
        }

        /* Baudrate of the DWM1001-DEV by Default */
        cfsetospeed(&tty, B460800);
        cfsetispeed(&tty, B460800);

        tty.c_cflag |= (CLOCAL | CREAD);    //< enable receiver, set local mode
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;                 //< 8 data bits
        tty.c_cflag &= ~PARENB;             //< no parity
        tty.c_cflag &= ~CSTOPB;             //< 1 stop bit
        tty.c_cflag &= ~CRTSCTS;            //< no flow control

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //< raw input, dont echo input
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);         //< no flow control
        tty.c_oflag &= ~OPOST;                          //< raw output, disable output processing

        tty.c_cc[VMIN] = 1; //< read at least 1 character, block until available. May be an issue...
        tty.c_cc[VTIME] = 0; //< no timeout

        tcsetattr(_fd, TCSANOW, &tty);
    }

    void readLoop() {
        std::string line;

        /**
         * A JSON Message with 4 Anchors is expected to be in the format:
         * {
         *   "utime": "timestamp", 
         *   "seq": "sequencenumber",
         *   "uid": "TAG short addr",
         *   "rngs": [
         *     {"uid": "anchor1", "rng": 1000},
         *     {"uid": "anchor2", "rng": 1000},
         *     {"uid": "anchor3", "rng": 1000},
         *     {"uid": "anchor4", "rng": 1000},
         *   ]
         * }
         */
        char buf[256];

        while (_running) {
            ssize_t len = read(_fd, buf, sizeof(buf) - 1);
            if (len > 0) {
                buf[len] = '\0';
                for (ssize_t i = 0; i < len; ++i) {
                    if (buf[i] == '\n') { //< read until newline
                        if (!line.empty()) {
                            processLine(line);
                            line.clear();
                        }
                    } else {
                        line += buf[i];
                    }
                }
            }
        }
    }

    void processLine(const std::string& line) {
        try {
            auto obj = json::parse(line);
            if (!obj.contains("uid") || !obj.contains("rngs"))
                return;

            std::lock_guard<std::mutex> lock(_mutex);
            _latest = obj; //< store the latest valid JSON object
        } catch (...) {
            // ignore malformed JSON
        }
    }

    std::string _devPath;
    std::atomic<bool> _running;
    std::thread _thread;
    std::mutex _mutex;
    std::unordered_map<uint16_t, size_t> _anchorMap;
    json _latest;
    int _fd;
};