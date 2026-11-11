# MIC3117 – Laser turret robot with on-device object detection

Integration project (MIC3117) by Ouimet & LeBlanc.

An ESP32-CAM runs an Edge Impulse FOMO model that detects a target ("bug"). Each detected
bounding box is sent over ESP-NOW to an ESP32-C3 Mini, which measures the distance with a
VL53L4CX time-of-flight sensor, computes pan/tilt angles (with camera-to-laser parallax
correction), aims two servos and turns on the laser. A separate ESP32-CAM drives the car
and streams video to a web remote.

```
ESP32-CAM (FOMO detection) ──ESP-NOW──▶ ESP32-C3 Mini (LiDAR + pan/tilt servos + laser)

ESP32-CAM (car) ──Wi-Fi──▶ browser (video stream + drive controls)
```

## Repository layout

```
firmware/
├── camera/esp32_camera/                 ESP32-CAM: FOMO inference, sends boxes via ESP-NOW
├── turret/esp32_c3_turret_lidar/        ESP32-C3 Mini: receives boxes, LiDAR ranging, aiming, laser
├── car/VideoCarRemote/                  ESP32-CAM car: MJPEG stream + web drive controls
└── prototypes/
    ├── esp32_c3_turret_lidar_dev/       earlier turret version (full 3D projection model)
    └── turret_basic/                    first single-servo ESP-NOW test
lib/
└── ei-edge-detect-arduino-1.0.9.zip     exported Edge Impulse model (192x192, label "bug")
```

Each sketch is in a folder with the same name, as the Arduino IDE requires.

## Building

All sketches use the Arduino IDE with the ESP32 Arduino core. The turret uses the
`esp_now_recv_info_t` receive callback, which needs ESP32 core 3.x.

**Camera (`esp32_camera`)** – board: AI Thinker ESP32-CAM (PSRAM enabled).
Install the model with *Sketch → Include Library → Add .ZIP Library…* and pick
`lib/ei-edge-detect-arduino-1.0.9.zip`. Set `broadcastAddress` to the MAC address of the
turret board (it prints its MAC on boot).

**Turret (`esp32_c3_turret_lidar`)** – board: ESP32-C3 Mini. Libraries: `ESP32Servo`,
`STM32duino VL53L4CX`.

| Function        | GPIO |
|-----------------|------|
| I2C SDA / SCL   | 5 / 4 |
| Servo pan / tilt| 18 / 19 |
| Laser           | 7 |
| LiDAR power     | 3 |

Calibration constants (camera FOV, camera-to-laser offsets, servo offsets) are at the top of
the sketch.

**Car (`VideoCarRemote`)** – board: AI Thinker ESP32-CAM. Fill in `ssid` and `password`
before flashing, then open the IP address printed on the serial monitor. Motor pins 12–15,
LED on GPIO 4.

## Credits

- `esp32_camera` is based on the Edge Impulse ESP32 camera example (MIT licence, header kept).
- `VideoCarRemote` is based on the Keyestudio ESP32-CAM robot car example.
