# US Geological Survey LoRa Stream Gauge Sensor

This repo contains the firmware for the USGS LoRa Stream Gauge Sensor.

## Getting Started

* Before using this repository, make sure to set up the ESP-32 IDF toolchain to properly compile ESP-32 apps.
* Pull down all submodules for this project `git submodule update --init -recursive`
* Configure the LoRaWAN network keys and device IDs in `app/main/config.h`
* Compile and flash the app to your device `make flash monitor ESPPORT=<path_to_device>`

## Repository Layout

The repo is divided into the following sections:

* ***app*** - contains the main application program
* ***lib*** - contains driver libraries for peripheral components
* ***boards*** - contains the specific configurations for the board
* ***esp-idf*** - contains the reference ESP-IDF repo for all ESP-32 related libraries.
