# ADS LoRa RF Project

## Introduction

This project uses Rust Embassy combined with a Heltec WiFi LoRa 32 V3.
The Heltec WiFi LoRa 32 V3 board has a SX1262 LoRa node chip (915Mhz) and an ESP32S3 microcontroller.

The Goal of this project is to be placed in the ADS section of our competition rocket.

It connects over UART to the ADS system, reciveing flap deployment angle and battery voltage data.
Connected to the LoRa32 is a Ublox NEO-M9N GPS.

This connects over a LoRa mesh network combined with the payload node and the ground station.

Data will be sent in the APRS format.
Specific APRS format is TBD (Waiting for IREC response)

Link to APRS format: [APRS](http://www.aprs.org/doc/APRS101.PDF)

## About Rust

Why use Rust over C or C++?

Rust not only provides the performance of C and C++ but also provides memory safety and thread safety.

But way more importantly, Rust provides the `Cargo` package manager. This allows for easy dependency management and building.

This means no more CMake or Makefiles. Which has been the biggest hurdle when introducing new members to our projects.

The downside is that Rust code trades a lot of the simplicity of C and C++ for safety and performance which requires a steeper learning curve.

Often this is seen as (in embedded rust only for the most part) insane Types and Lifetimes errors.

This can be alleviated some by using only Pi Pico or STM32 HAL crates as those are directly supported by most libraries. While ESP32 are not... But this is not always possible.

## Building

### Install the Dependence

```bash
cargo install espflash
```

### Build the Project

```bash
cargo build --release
```

## Flashing

```bash
cargo run --release
```
