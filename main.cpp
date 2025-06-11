#include "dwm_uart_reader.hpp"
#include <unistd.h>
#include <fstream>
#include <vector>
#include <numeric>
#include <cmath>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>


int main() {
    int sockfd;
    DWMUartReader reader("/dev/ttyACM0", "../config.json"); // Adjust the device path as needed

    if (!reader.start()) {
        std::cerr << "Failed to start UART reader." << std::endl;
        return 1;
    }

    /* (0,0) Mittelpunkt */
    const pos POS_ANCHOR_1 = { .x = -0.25, .y = 0.25, .z = 0.0 };
    const pos POS_ANCHOR_2 = { .x = 0.25, .y = 0.25, .z = 0.0 };
    const pos POS_ANCHOR_3 = { .x = 0.25, .y = -0.25, .z = 0.0 };
    const pos POS_ANCHOR_4 = { .x = -0.25, .y = -0.25, .z = 0.0 };

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(6969); // Choose a port

    int opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt(SO_REUSEADDR) failed");
    }

    if (bind(sockfd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    distances ds = (distances) { 0.0, 0.0, 0.0, 0.0 };
    pos result = (pos) { 0.0, 0.0, 0.0 };

    listen(sockfd, 1);
    printf("Waiting for client to connect...\n");

    int client_fd = accept(sockfd, NULL, NULL);
    if (client_fd < 0) {
        perror("Accept failed");
        exit(EXIT_FAILURE);
    }
    printf("Client connected.\n");

    while (true) {
        ds = reader.get_latest_distances_to_anchors();

        result = coords_calc(
            ds.d1, ds.d2, ds.d3, ds.d4,
            POS_ANCHOR_1,
            POS_ANCHOR_2,
            POS_ANCHOR_3,
            POS_ANCHOR_4
        );

        char buffer[128];
        snprintf(buffer, sizeof(buffer), "%.3f,%.3f,%.3f\n", result.x, result.y, 0.0f);
        send(client_fd, buffer, strlen(buffer), 0);

        // Print the calculated position
        fprintf(stdout, "Calculated Position: x=%.2f, y=%.2f, z=%.2f\n",
               result.x, result.y, 0.0f);

        usleep(250000);
    }
    
    reader.stop();
    std::cout << "UART reader stopped." << std::endl;

    return EXIT_SUCCESS;
}
