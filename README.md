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
- Break up different cpp files into separate classes
- mutexes for all calculation and publishing
- .02m/s speed ssems like minimum. add that dead band
- take minimum pulses for a side to remove outliers
- Timeout if no twist received for some period of time and stop motors
- Publish Odom
- IMU Retries
- Set status string for diagnostic
- handle twist properly
- don't arm motors until ready
- Mutex on Odom pub vs calculation
- if a motor is not getting any pulses after some time, raise some kind of error and stop

- clear petg led ring fresnel lens
- 3m screws for led cover
- IMU AD0 pin connect to address select
- IMU calibration: https://github.com/mattwilliamson/deepdrive_micro/commit/6a417c4e63e32671b85648450a2366626a14d2dd#diff-97ccfa9770f9d7f64ec98c8791e52b315f7ed2c2cf4eb915823ea091dc1241d6R196 for accelerometer + gyro, just read a bunch of times and average it. that's the bias to set
- IMU Shock detected
- IMU Not working on PCB - 2 sda and 2 scl?
- IMU Interrupts 
- IMU Quaternion accuracy header

- tpu spacer gasket for pcb screws
- tpu gasket for behind camera
- publish twistwithcovariancestamped instead of odom?
- robot_localization
- timeout i2c for imu so it doesn't freeze everything
- power on self test (thinking about IMU here)
- scaling imu output (Gs) - we don't need 8 Gs worth of scale
- imu is sampling at 100hz, but our control loop is 50hz - not a problem if we do onboard dmp

- param server for pid, speed, etc - mostly coded, just needs to fix the build
- use flash to save params
- use mutex to lock status? https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#mutex

- Perhaps 2 nodes?
- speed up publishing with static memory https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/static_type_handling/main.c https://docs.vulcanexus.org/en/humble/rst/tutorials/micro/memory_management/memory_management.html#entity-creation



### Motor Feedback Loop
- interpolators for counting pulses? or pio
- pid controller on interpolator?
- back right encoder is giving some noisy pulse counts (there was a solder bridge)
- tune pid controller - output speed might not be correct (might not be estimating speed accurately)

- Driving high the SMPS mode pin (GPIO23), to force the power supply into PWM mode, can greatly reduce the - inherent ripple of the SMPS at light load, and therefore the ripple on the ADC supply. This does reduce the - power efficiency of the board at light load, so the low-power PFM mode can be re-enabled between infrequent ADC - measurements by driving GPIO23 low once more. See Section 4.4.

