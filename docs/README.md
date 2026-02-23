
Paste this:

```markdown
# 📘 AutoSpray Firmware Documentation

This folder contains detailed documentation about:

- RTOS architecture
- Task design
- Inter-task communication
- Safety mechanisms
- Watchdog strategy
- OTA update design
- SD card buffering logic

---

# 📌 Key Design Principles

1. Deterministic scheduling
2. No blocking infinite waits
3. Queue overwrite for sensor streams
4. Separation of drivers and services
5. Configurable constants in config/ folder

---

# 🧠 FreeRTOS Concepts Used

- Task
- Queue
- Event Group
- Critical Section
- Core Pinning
- Preemptive Scheduler
- Priority based execution
- Stack sizing

---

# 🔒 Safety Features (Planned)

- Watchdog supervisor task
- OTA rollback
- SD fail-safe logging
- MQTT reconnect state machine

---

# 📊 Task Priority Table

| Task         | Priority |
|-------------|----------|
| SprayTask   | 6 |
| LidarTask   | 5 |
| ProxiTask   | 5 |
| ButtonTask  | 4 |
| DebugTasks  | 2-3 |

---

# 🧱 Design Pattern

Driver → Queue → Service

Never:
- Call driver directly from service
- Block forever inside tasks
- Hardcode configuration values

---

# 📌 Versioning

Semantic versioning recommended:
v1.0.0 → Initial RTOS architecture
v1.1.0 → GPS + MQTT
v1.2.0 → OTA + SD