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
- diagnostic with timing for each core loop
- convert joint state to radians for position
- refactor publishers into separate files
- refactor pid controller
- publish odom
- handle twist messages
- icm20948 has a dmp onboard. use sparkfun portable library: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
    rewrite driver to use their sdk instead of generating a quaternion on cpu @ 100+hz
- convert speed to m/s for cmd
- subscribe to cmd commands
- timeout i2c for imu so it doesn't freeze everything
- power on self test
- scaling imu output (Gs)
- imu is sampling at 100hz, but our control loop is 50hz
- adc_ref not connected.
- add icm20948 to pcb
- check ADC_VREF == 3.3 V
- if a motor is not getting any pulses after some time, raise some kind of error and stop
- diagnostic messages
- param server for pid, speed, etc
- joint state on mcu?
- publish temperature from imu?
- use flash to save params
- use mutex to lock status? https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#mutex



### Motor Feedback Loop
- interpolators for counting pulses? or pio
- pid controller on interpolator?
- back right encoder is giving some noisy pulse counts (there was a solder bridge)
- tune pid controller - output speed might not be correct (might not be estimating speed accurately)


### ADC
- ADC_AVDD should be decoupled with a 100nF capacitor close to the chip’s ADC_AVDD pin.
- ADC_AVDD can use from the same power source as the digital IO supply (IOVDD)
- IOVDD, VREG_VIN = 3.3v 
- reset button on pin 30 RUN that pulls it low
- Driving high the SMPS mode pin (GPIO23), to force the power supply into PWM mode, can greatly reduce the - inherent ripple of the SMPS at light load, and therefore the ripple on the ADC supply. This does reduce the - power efficiency of the board at light load, so the low-power PFM mode can be re-enabled between infrequent ADC - measurements by driving GPIO23 low once more. See Section 4.4.

