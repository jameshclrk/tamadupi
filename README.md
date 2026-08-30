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
idf.py -p /dev/cu.usbmodem2101 flash monitor
```

Use a different serial port if the board appears under another device name.

Weather data comes from Open-Meteo and refreshes every 30 minutes. The header
shows `Wi-Fi setup` until `main/secrets.h` contains a network name. Real
credentials must stay in `main/secrets.h`, which is ignored by Git.

## Expected Result

The AMOLED shows a small animated character:

- Tilt the board left and right to make Mochi lean and look around.
- Move the board side to side to slide Mochi around; Mochi squishes on impact with the screen edges.
- Walking with Tamadupi counts steps and gradually improves Health. The first
  three cadence-consistent impacts confirm a walk, which filters out tilting,
  isolated bumps, and fast shaking.
- Give the board a gentle shake to trigger Mochi's surprised expression.
- Nearby BLE advertisers raise Social over time. Addresses are kept only in RAM,
  are never logged, and expire after ten minutes.
- Current temperature and conditions appear in the header. Sun, clouds, rain,
  snow, and storms change the scene and Mochi's mood.
- Health and Social survive reboots. Low scores give Mochi tired or lonely
  expressions and hints about what it needs.
- Leave it still and Mochi breathes and blinks.

The status pill normally reports weather state. If it shows `no IMU`, check the
serial monitor for the I2C or QMI8658 error.

## Tuning

Animation thresholds, sampling rate, and UI colors are grouped near the top of
`main/main.c`. Pedometer timing and acceleration thresholds are in
`main/step_tracker.c`.

Run the host-side pedometer checks with:

```bash
cc -std=c11 -Wall -Wextra -Werror -I main \
  main/step_tracker.c tests/step_tracker_test.c -lm \
  -o /tmp/tamadupi-step-test && /tmp/tamadupi-step-test
```

Never put real credentials in `secrets.example.h`.
