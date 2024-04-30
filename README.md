# deepdrive_micro

ros micro interface to raspberry pi pico

Interfaces:
- Motors
- Motor rotary encoder
- LED?
- GPS?
- LIDAR?

```sh
git submodule update --init --recursive
```

## Mount RPI Pico

*Hold boot button when plugging in*

**TODO: probably can use `picotool` to make this easier**

```sh
sudo mkdir /mnt/sda1
sudo vi /etc/fstab

# fstab
/dev/sda1 /mnt/sda1 vfat defaults 0 0

sudo mount /mnt/sda1
```

## Build & Flash

TODO: for param server .vscode/settings.json
Param server is not working. Calling spin on the node returns an error
```json
    "cmake.configureArgs": [
        "-DRMW_UXRCE_MAX_NODES=1",
        "-DRMW_UXRCE_MAX_PUBLISHERS=21",
        "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=21",
        "-DRMW_UXRCE_MAX_SERVICES=6",
        "-DRMW_UXRCE_MAX_CLIENTS=0"
    ],
```
`libmicroros/include/rmw_microxrcedds_c/config.h` should get updated when building

```sh
make dockershell
colcon build --cmake-args -DPICO_BOARD=pico_w --packages-select deepdrive_micro

# Copy it to src so the host machine has access
cp build/deepdrive_micro/deepdrive_micro.uf2 src/deepdrive_micro

# Copy it to rpi pico
sudo mount /mnt/sda1
sudo cp src/deepdrive_micro/deepdrive_micro.uf2 /mnt/sda1
```

## Run micro ros Agent
```sh
mamba activate ros_env
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600

ros2 topic echo deepdrive_micro/pulses
```

## Publish speed
Min/Max int16: +/- 32000

```sh
ros2 topic pub --once deepdrive_micro/cmd control_msgs/msg/MecanumDriveControllerState '{
    "front_left_wheel_velocity":  0.5,
    "front_right_wheel_velocity": 0.5,
    "back_left_wheel_velocity":   0.5,
    "back_right_wheel_velocity":  0.5
}'

ros2 topic pub --once deepdrive_micro/cmd control_msgs/msg/MecanumDriveControllerState '{
    "front_left_wheel_velocity":  0,
    "front_right_wheel_velocity": 0,
    "back_left_wheel_velocity":   0,
    "back_right_wheel_velocity":  0
}'

ros2 topic pub --once deepdrive_micro/cmd control_msgs/msg/MecanumDriveControllerState '{
    "front_left_wheel_velocity":  -0.5,
    "front_right_wheel_velocity": -0.5,
    "back_left_wheel_velocity":   -0.5,
    "back_right_wheel_velocity":  -0.5
}'
```

TODO: 1000 stops
2000 is slow
3000 is a good pace - maybe starting point?

## Testing pulse speed
// 127 revs, 93156 pulses, 60s, signal = 100%
// one wheel is 2.2 rev/s another is 2.95
Slower results in fewer or missed pulses?
throttle@2000: 34826/50 = 696 pulses per revolution
thottle@32767: 93156/127 = 734 pulses per revolution

### 60s @ 3000:
front_left_wheel_velocity   7710
back_left_wheel_velocity    10491
back_right_wheel_velocity   8406
front_right_wheel_velocity  10891

### 60s @ 32767
front_left_wheel_velocity   87269
back_left_wheel_velocity    87962
back_right_wheel_velocity   87201
front_right_wheel_velocity  89018


# TODO
- Emergency stop for twist mux - in case of cliffs or whatnot - publish std_msgs::Bool to /e_stop
- Add config for open loop and bypass pulse counter
- if a motor is not getting any pulses after some time, raise some kind of error and stop
- take minimum pulses for a side to remove outliers
- Sonar for front and back
- Odom is off. check each source independently. might need to fix pulse counters somehow
- need to tune PID controller under load
- Watchdog for restarts
- Base class for publishers
- Speed up core0 with static memory https://docs.vulcanexus.org/en/humble/rst/tutorials/micro/memory_management/memory_management.html#entity-creation
- Mutexes don't work across cores. Use FIFO to tell core0 which step core1 is on: https://github.com/raspberrypi/pico-examples/blob/master/multicore/multicore_fifo_irqs/multicore_fifo_irqs.c
- Add status for each publisher to diagnostic
- Fix up error handling throughout and status manager
- IMU SPI?
- Average IMU Readings
- IMU Retries
- Set status string for diagnostic
- average battery voltage

- move lidar back to make room for usb
- OR move jetson to passenger side with riser and rotate clockwise 90 degrees
- clear petg led ring fresnel lens
- 3m screws for led cover
- IMU AD0 pin connect to address select
- IMU Shock detected
- IMU Interrupts 
- IMU Quaternion accuracy header

- tpu spacer gasket for pcb screws
- tpu gasket for behind camera
- publish twistwithcovariancestamped instead of odom?
- timeout i2c for imu so it doesn't freeze everything
- power on self test (thinking about IMU here)
- scaling imu output (Gs) - we don't need 8 Gs worth of scale

- param server for pid, speed, etc - mostly coded, just needs to fix the build
- use flash to save params



### Motor Feedback Loop
- interpolators for counting pulses? or pio
- pid controller on interpolator?
- back right encoder is giving some noisy pulse counts (there was a solder bridge)


