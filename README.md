# microros_ruka
#Последовательность установки:
#Распберри
1. Используя Raspberry Imager накатить на SD Ubuntu Server 22.04+
2. Зайти на распберри и установить ROS2 Humble стандартным способом
3. Включить CAN (описано тут https://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/#software)
    3.1 Добавить в раздел [All] файла /boot/firmware/config.txt
        dtoverlay=seeed-can-fd-hat-v2
    3.2 Добавить в автозагрузку, например в .bashrc
        sudo ip link set can0 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on