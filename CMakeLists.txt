cmake_minimum_required(VERSION 3.10)
project(dwm_uart_reader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Suche nach dem nlohmann_json-Package
find_package(nlohmann_json 3.12.0 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

add_executable(dwm_uart_reader
    main.cpp
)

# Linke pthread (für std::thread) und nlohmann_json
target_link_libraries(dwm_uart_reader
    PRIVATE
        Eigen3::Eigen
        pthread
        nlohmann_json::nlohmann_json
)