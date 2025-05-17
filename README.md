# ffi6_uwb_drone_localization

## 🛠️ Dependencies

To build this project, ensure the following tools and libraries are installed on your system:

### ✅ Required Packages

| Dependency         | Minimum Version | Installation Command (Ubuntu/Debian)                          |
|--------------------|------------------|---------------------------------------------------------------|
| **CMake**          | 3.10+            | `sudo apt install cmake`                                     |
| **G++ / Clang++**  | X                | `sudo apt install build-essential`                           |
| **Eigen**          | 3.4.0+           | `sudo apt install libeigen3-dev`                             |
| **GPIOD**          | 1.6.3-1+b        | `sudo apt install libgpiod-dev`                             |
---
Clone this repository with submodules

```bash
git clone --recurse-submodules https://github.com/j-peller/ffi6_uwb_drone_localization.git
```
---

Make sure to run CMake from within the `build/` directory in `Drone/dwm1000_control`

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

Or build using docker buildx from the repository root direction:

```bash
docker buildx build --platform linux/arm64 --build-arg BUILD_TYPE=Release -t rpi5_dwm:arm64 --load . --output type=local,dest=Drone/dwm1000_control/build
```

The binary can be found in Drone/dwm1000_control/build