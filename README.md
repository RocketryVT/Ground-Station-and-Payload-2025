# Ground-Station-and-Payload-2025

Ground Station for Rocketry@VT for Spaceport 2025

## Getting Started

## Clone

**Note: You must initialize the git submodules prior to utilizing CMake for a proper build.**

```shell
git clone https://github.com/RocketryVT/Ground-Station-and-Payload-2025.git
cd Ground-Station-and-Payload-2025/
git submodule update --init --recursive
```

## Download

### Payload

- ### Windows

  - Chocolatey - <https://chocolatey.org/install>
    - in an admin terminal run the following
      - `choco install cmake ninja mingw`
  - <https://developer.arm.com/downloads/-/gnu-rm>

## Build

### Payload

- ### Windows

  - Open a terminal in the payload directory
  - `cmake -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release`
  - `cmake --build build`
  