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

## Installation

### Setup device alias to `/dev/deepdrive_micro`

Check attributes

```sh
$ sudo dmesg
[434031.462057] usb 1-2.4.2: new full-speed USB device number 9 using tegra-xusb
[434031.578688] usb 1-2.4.2: New USB device found, idVendor=2e8a, idProduct=000a, bcdDevice= 1.00
[434031.578698] usb 1-2.4.2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[434031.578702] usb 1-2.4.2: Product: Pico
[434031.578706] usb 1-2.4.2: Manufacturer: Raspberry Pi
[434031.578710] usb 1-2.4.2: SerialNumber: 454741505A85844A
[434031.616606] cdc_acm 1-2.4.2:1.0: ttyACM0: USB ACM device
[434031.618299] usbcore: registered new interface driver cdc_acm
[434031.618303] cdc_acm: USB Abstract Control Model driver for USB modems and ISDN adapters

$ udevadm info -a -n /dev/ttyACM0 | grep serial
    ATTRS{serial}=="454741505A85844A"
    ATTRS{serial}=="3610000.xhci"
```

Setup alias

```sh
sudo apt install libudev-dev

cat << EOF | sudo tee /etc/udev/rules.d/99-deepdrive_micro.rules
ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE:="0777", SYMLINK+="deepdrive_micro"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify

```sh
$ ls -lah /dev/deepdrive_micro
lrwxrwxrwx 1 root root 15 Jul 29 07:37 /dev/deepdrive_micro -> bus/usb/001/024
```


## Run micro ros Agent
```sh
mamba activate ros_env
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

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
- Switch back to BNO08x on jetson
- Fix battery publisher
- Fix diagnostic string
- Wide angle camera taking up lots of cpu https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera
- ldlidar taking lots of cpu
- Try other laser publisher https://github.com/ldrobotSensorTeam/ldlidar_stl_ros/tree/master
- Try other laser odom https://wiki.ros.org/rf2o
- depth_image_to_laserscan https://answers.ros.org/question/393773/slam-toolbox-message-filter-dropping-message-for-reason-discarding-message-because-the-queue-is-full/
- Fix front sonar
- Some kind of exception handler so that when assert fails, it stops the motors and flashes the lights
- Motor must be set to 0 for a couple seconds before moving
- Turn off motor PWM when stopped for a period of time to avoid the beeping
- Remove pulse counter hardware filters
- Watchdog for restarts
- Fix pins for led ring
- Route pulse counter wires away from motors
- Fix sonar mount
- Holder for speaker
- Remount jetson
- Read IMU on interrupt to update orientation, but still publish same frequency
- Convert sonar scan to LaserScan so Nav2 can use it?
- Add frames to urdf for sonar
- Emergency stop for twist mux - in case of cliffs or whatnot - publish std_msgs::Bool to /e_stop
- Add config for open loop and bypass pulse counter
- if a motor is not getting any pulses after some time, raise some kind of error and stop
- take minimum pulses for a side to remove outliers
- Sonar for front and back
- Odom is off. check each source independently. might need to fix pulse counters somehow
- need to tune PID controller under load
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


