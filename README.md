# open-heating/firmware

ESP32-based heating controller firmware for residential heating systems.

## Overview

This firmware runs on a single ESP32 and controls up to 8 heating circuits independently. Each circuit consists of a temperature sensor (DS18B20 or PT1000) and a 3-point actuator (e.g., ESBE ARA600 series). The controller operates fully standalone — if the Raspberry Pi hub goes offline, heating continues with last-known setpoints.

### Key features

- **PID control** with 3-point actuator output (open/close/stop)
- **Weather-compensated** heating curve (outdoor temp → supply temp setpoint)
- **Dual sensor support**: DS18B20 (1-Wire) and PT1000 (via MAX31865 SPI)
- **MQTT** for telemetry, setpoint changes, and configuration
- **OTA updates** over WiFi
- **Watchdog** with automatic restart on hang
- **Reference run** on startup for actuator position calibration

## Hardware requirements

| Component | Per circuit | Total (4 circuits) |
|---|---|---|
| ESP32 DevKit v1 | — | 1x |
| DS18B20 (waterproof) | 0–1x | as needed |
| PT1000 + MAX31865 breakout | 0–1x | as needed |
| 2-channel relay module (230V, optocoupler) | 1x | 4x |
| ESBE ARA661 actuator (230V, 3-point) | 1x | 4x |
| DS18B20 outdoor sensor | — | 1x |
| 4.7kΩ pullup resistor (1-Wire bus) | — | 1x |

See [open-heating/hardware](https://github.com/open-heating/hardware) for wiring diagrams and full BOM.

## Building

### Prerequisites

- [PlatformIO CLI](https://platformio.org/install/cli) or PlatformIO IDE
- USB cable for initial flash

### Build & upload

```bash
# Build
pio run

# Upload via USB
pio run --target upload

# Upload via OTA (after initial flash)
pio run --target upload --upload-port <esp32-ip>

# Monitor serial output
pio device monitor -b 115200
```

## Configuration

Configuration is stored as JSON in SPIFFS and can be updated via MQTT. See [docs/configuration.md](docs/configuration.md) for details.

Example `config.json`:
```json
{
  "wifi": {
    "ssid": "your-network",
    "password": "your-password"
  },
  "mqtt": {
    "broker": "192.168.1.100",
    "port": 1883,
    "prefix": "open-heating"
  },
  "outdoor_sensor": {
    "type": "ds18b20",
    "address": "28-01234567abcd"
  },
  "circuits": [
    {
      "name": "Erdgeschoss",
      "sensor": { "type": "ds18b20", "address": "28-aabbccddeeff" },
      "actuator": {
        "pin_open": 25,
        "pin_close": 26,
        "run_time_seconds": 60
      },
      "pid": { "kp": 2.0, "ki": 0.1, "kd": 0.5 },
      "heating_curve": {
        "outdoor_min": -15,
        "outdoor_max": 20,
        "supply_at_min": 55,
        "supply_at_max": 25
      }
    }
  ]
}
```

## MQTT topics

| Topic | Direction | Description |
|---|---|---|
| `open-heating/circuit/{n}/temperature` | → broker | Current temperature (°C) |
| `open-heating/circuit/{n}/setpoint` | ← broker | Target temperature (°C) |
| `open-heating/circuit/{n}/valve_position` | → broker | Estimated valve position (0–100%) |
| `open-heating/outdoor/temperature` | → broker | Outdoor temperature (°C) |
| `open-heating/status` | → broker | Controller status (online/offline) |
| `open-heating/config` | ↔ | Configuration read/write |

## Project structure

```
firmware/
├── src/
│   ├── main.cpp              # Setup + main loop
│   ├── control_loop.h/.cpp   # PID controller + 3-point output
│   ├── sensor_manager.h/.cpp # DS18B20 + PT1000 abstraction
│   ├── heating_curve.h/.cpp  # Weather-compensated setpoint calc
│   ├── actuator_driver.h/.cpp# 3-point relay driver with safety
│   ├── mqtt_client.h/.cpp    # MQTT pub/sub
│   ├── ota_updater.h/.cpp    # ArduinoOTA wrapper
│   └── config.h/.cpp         # JSON config in SPIFFS
├── test/                     # Unit tests (PlatformIO native)
├── platformio.ini
└── README.md
```

## Safety

- Actuator interlock: `pin_open` and `pin_close` are never HIGH simultaneously
- Watchdog: 10s timeout, automatic restart
- Relay fail-safe: on ESP32 restart, all relays default to OFF (valves hold position)
- Reference run: on startup, drives each valve fully closed to establish 0% baseline
- Configurable min/max supply temperature limits

## License

Apache License 2.0 — see [LICENSE](LICENSE).
