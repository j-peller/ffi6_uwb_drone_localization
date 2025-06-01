# Use an ARM64 Ubuntu base image
FROM arm64v8/ubuntu:24.04 AS build

# Set environment variables to non-interactive (useful for automated builds)
ENV DEBIAN_FRONTEND=noninteractive

# Update and install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libgpiod-dev \
    libinih-dev \
    libwebsockets-dev \
    git

# Set working directory when copying files to include shared folder
WORKDIR /

# Copy your source code into the container
COPY . .

# Set working directory to the location of dwm1000_control's CMakeLists.txt
WORKDIR /Drone/dwm1000_control

# Configure and build your project
RUN cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} . && make -j4

# Stage 2: Export the final binary
FROM scratch AS export-stage
# Copy the final binary from the build stage.
COPY --from=build /Drone/dwm1000_control/dwm1000_control /dwm1000_control