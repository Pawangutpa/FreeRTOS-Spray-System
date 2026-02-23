# 🏗 System Block Diagram

LiDAR (CAN)
    ↓
LidarTask
    ↓ (Queue)
SprayTask

Flow + Nut Sensors
    ↓
ProxiTask
    ↓ (Queue)
SprayTask (future offset logic)

Button
    ↓
ButtonTask
    ↓ (EventGroup)
SprayTask

Future:
EC200U → MQTTTask → SDTask → OTATask