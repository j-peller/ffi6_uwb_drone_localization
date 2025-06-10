#include "dwm_uart_reader.hpp"

int main() {
    DWMUartReader reader("/dev/ttyACM0", "../config.json"); // Adjust the device path as needed
    if (!reader.start()) {
        std::cerr << "Failed to start UART reader." << std::endl;
        return 1;
    }

    std::cout << "UART reader started. Press Enter to stop..." << std::endl;

    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        distances_t distances = reader.get_latest_distances_to_anchors();
        std::cout << "Distances: "
                  << "Anchor 1: " << distances.d1 << " m, "
                  << "Anchor 2: " << distances.d2 << " m, "
                  << "Anchor 3: " << distances.d3 << " m, "
                  << "Anchor 4: " << distances.d4 << " m" << std::endl;
    }

    reader.stop();
    std::cout << "UART reader stopped." << std::endl;

    return 0;
}
