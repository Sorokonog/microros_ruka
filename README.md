# microros_ruka
#Последовательность установки:
#Распберри
1. Используя Raspberry Imager накатить на SD Ubuntu Server 22.04+
2. Зайти на распберри и установить ROS2 Humble-ROS-base стандартным способом
    2.1 Настроить setup.bash стандартным образом
3. Включить CAN (описано тут https://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/#software)
    3.1 Добавить в раздел [All] файла /boot/firmware/config.txt
        dtoverlay=seeed-can-fd-hat-v2
    3.2 Добавить в автозагрузку, например в .bashrc
        sudo ip link set can0 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on

4. mkdir uros_ws && cd uros_ws (Устанавливаем микророс описано тут: https://github.com/micro-ROS/micro_ros_setup)
5. git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
6. rosdep update && rosdep install --from-paths src --ignore-src -y
7. colcon build
8. source install/local_setup.bash
9. ros2 run micro_ros_setup create_agent_ws.sh
10. ros2 run micro_ros_setup build_agent.sh
11. cd
12. git clone https://github.com/Sorokonog/microros_ruka.git (клонировать репо в домашнюю директорию)
13. cp /microros_ruka/custom_agent.cpp /uros_ws/build/micro_ros_agent/agent/src/xrceagent/examples/custom_agent/custom_agent.cpp (копируем кастомный транспорт для CAN)
14. В файле /home/pi/uros_ws/build/micro_ros_agent/agent/src/xrceagent/CMakeLists.txt поменять значение параметра option(UAGENT_BUILD_USAGE_EXAMPLES "Build Micro XRCE-DDS Agent built-in usage examples" OFF) на ON (заставляем собирать custom_agent.cpp)
15. rm -rf /home/pi/uros_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeCache.txt
16. cd /uros_ws/build/micro_ros_agent/agent/src/xrceagent-build
17. make -j1 (пересобираем все занимает около 20-30 минут)
18. cd && ./home/pi/uros_ws/build/micro_ros_agent/agent/src/xrceagent-build/examples/custom_agent/CustomXRCEAgent (Запускаем агента)