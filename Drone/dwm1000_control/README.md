# DWM1000 Controller

## Overview
This project provides control and communication with the DWM1000 module using SPI.

## Prerequisites
- CMake (version 3.10 or higher)
- A C++17 compatible compiler
- Linux environment with SPI support

## Build Instructions
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd ffi6_uwb_drone_localization/Drone/dwm1000_control
   ```

2. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```

3. Configure the project:
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release
   ```

   You can replace `Release` with `Debug` for a debug build.

4. Build the project:
   ```bash
   make
   ```

5. The executable will be located in the `build` directory.

## Run Instructions
Ensure you have the necessary permissions to access the SPI device and GPIO pins. Then, run the executable:
```bash
./dwm1000_control
```

## Cleaning the Build
To clean the build directory, simply remove it:
```bash
rm -rf build
```

## Notes
- Update the `dw1000_dev_instance_t` configuration in `main.cpp` to match your hardware setup.
- Ensure the SPI device (`/dev/spidev0.0`) and GPIO pins are correctly configured and accessible.
