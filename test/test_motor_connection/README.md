# Test motor connection

This is a minimal package to test t-motor connection. This package is ignored by `colcon build` command.

## Build

To build the package using CMake, follow these steps:

Create a build directory:

```bash
cmake -S . -B build # If build directory has not been made
cd build
cmake --build .
```

## Test with Holybro

```bash
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```
