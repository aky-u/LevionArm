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
