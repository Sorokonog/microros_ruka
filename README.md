# microros_ruka
Последовательность установки:
## Распберри
# Ubuntu
* Используя Raspberry Imager накатить на SD `Ubuntu Server 22.04+`
# ROS
* Зайти на распберри и установить `ROS2 Humble-ROS-base` стандартным способом
* Настроить setup.bash стандартным образом
# CAN
* Включить CAN (описано тут https://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/#software)
* Добавить в раздел ***[All]*** файла `/boot/firmware/config.txt`
        `dtoverlay=seeed-can-fd-hat-v2`
* Добавить в автозагрузку, например в `.bashrc`
        `sudo ip link set can0 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on`
## MicroROS Agent
* `mkdir uros_ws && cd uros_ws` (Устанавливаем микророс описано тут: https://github.com/micro-ROS/micro_ros_setup)
* `git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup`
* `rosdep update && rosdep install --from-paths src --ignore-src -y`
* `colcon build`
* `source install/local_setup.bash`
* `ros2 run micro_ros_setup create_agent_ws.sh`
* `ros2 run micro_ros_setup build_agent.sh`
* `cd`
* `git clone https://github.com/Sorokonog/microros_ruka.git` (клонировать этот проект в домашнюю директорию)
* `cp /microros_ruka/custom_agent.cpp /uros_ws/build/micro_ros_agent/agent/src/xrceagent/examples/custom_agent/custom_agent.cpp` (копируем кастомный транспорт для CAN)
* В файле `/home/pi/uros_ws/build/micro_ros_agent/agent/src/xrceagent/CMakeLists.txt` поменять значение параметра `option(UAGENT_BUILD_USAGE_EXAMPLES "Build Micro XRCE-DDS Agent built-in usage examples" OFF)` на `ON` (заставляем собирать custom_agent.cpp)
* `rm -rf /home/pi/uros_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeCache.txt`
* `cd /uros_ws/build/micro_ros_agent/agent/src/xrceagent-build`
* `make -j1` (пересобираем все занимает около 20-30 минут)
* `cd && ./home/pi/uros_ws/build/micro_ros_agent/agent/src/xrceagent-build/examples/custom_agent/CustomXRCEAgent` (Запускаем агента)

## STM32
* Скачать STM CubeIDE и CubeMX (Если вы хотите использовать какие-то свои инструменты, то данный гайд направит вас по нужному пути, но возможно не все заработает из коробки, для STM Cube все оттестировано и работает)
* Создайте новый Cube MX проект (**!!!ВНИМАНИЕ!!!** именно MX, а не IDE)
* В `Project Manager` выберете `toolchain` - `Makefile`
* В `Project Manager -> Code Generator` выберете `Generate peripheral intitialization as a pair of '.c/.h' files per peripheral` (опционально)

## Общие настройки STM
* В `System Core -> RCC -> High Speed Clock (HSE)` выберете `Crystal/Ceramic Resonator` и Настройте тактирование в соответствии с характеристиками STM, примерно так: ![alt text](https://github.com/Sorokonog/microros_ruka/blob/main/img/clock.jpg?raw=true)
* В `System Core -> SYS -> Timebase Source` выберете `TIM1`
* `PA4` - GPIO_Output
* `PD2` - GPIO_Output
* `PC1` - GPIO_Output
* `PC7` - GPIO_Output

## Timers
* `PC6 - TIM3 CH1` `TIM3 Channel1` - PWM_Generation, `Prescaler` = 1, NVIC Settings включить `TIM3 global interrupt`

## IWDG
* Активируйте IWDG и сконфигурируйте его в соответсвии с картинкой: ![alt text](https://github.com/Sorokonog/microros_ruka/blob/main/img/IWDG.jpg?raw=true)

## DMA
* Активируйте DMA и сконфигурируйте его в соответсвии с картинкой: ![alt text](https://github.com/Sorokonog/microros_ruka/blob/main/img/DMA.jpg?raw=true)

### CAN
* В `Connectivity` выберете `CAN / CAN1`
* На экране настройки GPIO выберете `PB8 - CAN1 RX, PB9 - CAN1 TX`
9. Настройте CAN следующим образом: ![alt text](https://github.com/Sorokonog/microros_ruka/blob/main/img/CAN1.jpg?raw=true) 
* В `NVIC Settings` включите `CAN1 RX0 interrupt`

### I2C
* В `Connectivity` выберете `I2C1` ->  Mode `I2C`
* Проверьте что настройки по умолчанию не сбились: ![alt text](https://github.com/Sorokonog/microros_ruka/blob/main/img/I2C.jpg?raw=true) 
* На экране настройки GPIO выберете `PB6 - I2C1_SCL, I2C1_SDA`

### SPI
* В `Connectivity` выберете `SPI1` ->  Mode `Full-Duplex Master`
* Настройте `Parametr Settings` в соответствии с картинкой: ![alt text](https://github.com/Sorokonog/microros_ruka/blob/main/img/SPI.jpg?raw=true)

### FreeRTOS
* В `Middleware -> FREERTOS -> Interface` выберете `CMSIS_V2`
* В `Middleware -> FREERTOS -> Configuration -> Task and Queues -> Deafult Task` задайте `stack size` = 3000.
* В `Middleware -> FREERTOS -> Configuration -> Task and Queues -> Deafult Task` **TODO**

### Genrate code
* Финальная настройка должна выглядеть вот так: ![alt text](https://github.com/Sorokonog/microros_ruka/blob/main/img/PINOUT.jpg?raw=true)
* Кликните на `Generate Code`

## MicroROS библиотека

* В директории этого нового проекта `git clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git` (рекомендуется использовать WSL, но можно и PowerShell)
* В `Makefile` перед разделом `Build the application` вставьте:

```makefile
#######################################
# micro-ROS addons
#######################################
LDFLAGS += micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
C_INCLUDES += -Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include

# Add micro-ROS utils
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_time.c

# Set here the custom transport implementation
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c
```

* В `Makefile` в раздел `Sources`
```makefile
######################################
# source
######################################
# C sources вставьте:
C_SOURCES = \
...
Core/Src/syscalls.c \
...
```


### Перенос файлов проекта
* Перенесите файлы `main.c freertos.c iwdg.c syscalls.c tim.c` из директории в которую вы клонировали этот проект `microros_ruka` в директорию `Core/Src/` вашего проекта
* Перенесите файл `dma_transport.c` из директории в которую вы клонировали этот проект `microros_ruka` в директорию `micro_ros_stm32cubemx_utils/extra_sources/microros_transports/` вашего проекта
* Перенесите файлы `main.h` из директории в которую вы клонировали этот проект`microros_ruka` в директорию `Core/Inc/` вашего проекта

### Docker часть
* Установите Docker (данный гайд оттестирован при установке Docker в WSL, если вы устанавливаете Docker напрямую в Windows, адаптируйте команды соответственно)
* Из корневой директории проекта:
```bash
sudo service docker start
docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
```

## Cube IDE
* Выберете новый проект `New` -> `Makefile Projects from Existing Code`
* Выберете папку с вашим проектом
* Правой кнопкой на проект `Run as` -> `STM32 C/C++ Application`

