# ffi6_uwb_drone_localization

## 🛠️ Dependencies

To build this project, ensure the following tools and libraries are installed on your system:

### ✅ Required Packages

| Dependency         | Minimum Version | Installation Command (Ubuntu/Debian)                          |
|--------------------|------------------|---------------------------------------------------------------|
| **CMake**          | 3.10+            | `sudo apt install cmake`                                     |
| **G++ / Clang++**  | C++17+           | `sudo apt install g++`                                       |
| **Eigen**          | 3.4.0+           | `sudo apt install libeigen3-dev`                             |
| **GoogleTest**     | 1.16.0           | Automatically fetched using `FetchContent` in `CMakeLists.txt` |

---

Make sure to run CMake from the `build/` directory:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

Or build using docker buildx:

```bash
docker buildx build --platform linux/arm64 --build-arg BUILD_TYPE=Release -t rpi5_dwm:arm64 --load . --output type=local,dest=Drone/dwm1000_control/build
```

The binary can be found in Drone/dwm1000_control/build