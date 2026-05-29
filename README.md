# ROS2-STM32-Sync-Driver

Hlavnou úlohou tohto uzla je zber dát z IMU, ktorá je pripojená k STM32 BluePill. Dáta sú prijímané v binárnej forme cez UART a následne spracované týmto uzlom do vhodného formátu a posielané na topic /imu.
## Spustenie uzla
Po pripojení správne zapojeného STM32 BluePill spolu s IMU k Jetsonu je možné spustiť uzol nasledovne
```bash
ros2 run stm32_sync_driver stm32_sync_driver
```

V prípade chybovej hlášky je potrebné spustiť príkaz
```
sudo chmod 666 /dev/ttyUSB0
```
Dáta z IMU môžu byť následne zobrazené takto
```
ros2 topic echo /imu
```
