Open Serial Monitor:
    platformio device monitor -p /dev/ttyUSB0 -b 115200

Upload:
    platformio run -e esp32dev --target upload --upload-port /dev/ttyUSB0