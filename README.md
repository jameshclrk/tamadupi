# Tamadupi

[简体中文](README_ZH.md)

Tamadupi is a sensor-driven virtual character for the Waveshare ESP32-S3-Touch-AMOLED-1.8. Mochi reacts to motion, nearby BLE devices, and the weather outside.

## Hardware

- ESP32-S3-Touch-AMOLED-1.8 board
- USB data cable

## Build and Flash

Copy the local configuration template before building:

```bash
cp main/secrets.example.h main/secrets.h
```

Edit `main/secrets.h` with the fixed Wi-Fi credentials and location. This file is ignored by Git and its values are compiled directly into the firmware.

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial port for the board.

## Expected Result

The AMOLED shows a small animated character:

- Tilt the board left and right to make Mochi lean and look around.
- Move the board side to side to slide Mochi around; Mochi squishes on impact with the screen edges.
- General movement makes Mochi bounce and fills the motion meter.
- Give the board a gentle shake to trigger Mochi's surprised expression.
- Leave it still and Mochi breathes and blinks.

The status pill reports `IMU live` when the sensor is connected. If it shows `no IMU`, check the serial monitor for the I2C or QMI8658 error.

## Tuning

Motion thresholds, sampling rate, and UI colors are grouped near the top of `main/main.c` for quick experimentation. Never put real credentials in `secrets.example.h`.
