# 🚜 AutoSpray RTOS Firmware (ESP32-S3)

Industrial-grade smart spraying system firmware built using:

- ESP32-S3
- FreeRTOS (Preemptive Scheduler)
- CAN (TWAI) LiDAR
- Proximity + Flow sensors
- Autonomous / Manual mode
- Event Groups
- Queue-based architecture

---

# 🏗 System Architecture

The firmware is built using layered architecture:

Application Layer (Services)
↓
RTOS Layer (Tasks, Queues, Events)
↓
Driver Layer (CAN, Proxi, Button)
↓
HAL (ESP32 / TWAI / GPIO)

---

# 🧠 RTOS Design

- Preemptive scheduling
- Queue length = 1 for sensor streams
- EventGroup for system mode
- Core pinned tasks (ESP32-S3 dual core)

---

# 📂 Folder Structure

src/
- drivers/      → Hardware drivers
- services/     → Business logic (Spray)
- rtos/         → Queue + Task management
- main.cpp      → Entry point

config/
- board_config.h
- spray_config.h
- system_config.h

docs/
- Architecture and design documentation

---

# 🚀 Build Instructions

```bash
pio run
pio run -t upload
pio device monitor