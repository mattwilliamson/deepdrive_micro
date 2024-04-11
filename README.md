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
    "front_left_wheel_velocity":  2000,
    "front_right_wheel_velocity": 2000,
    "back_left_wheel_velocity":   2000,
    "back_right_wheel_velocity":  2000
}'

ros2 topic pub --once deepdrive_micro/cmd control_msgs/msg/MecanumDriveControllerState '{
    "front_left_wheel_velocity":  0,
    "front_right_wheel_velocity": 0,
    "back_left_wheel_velocity":   0,
    "back_right_wheel_velocity":  0
}'

ros2 topic pub --once deepdrive_micro/cmd control_msgs/msg/MecanumDriveControllerState '{
    "front_left_wheel_velocity":  -2000,
    "front_right_wheel_velocity": -2000,
    "back_left_wheel_velocity":   -2000,
    "back_right_wheel_velocity":  -2000
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


# TODO
pid controller on mcu
max acceleration (curves?)
control loop in second core with led ring
logging
see if logs go to agent - no
diagnostic messages
param server for pid, speed, etc
convert speed to radians/s
param for loop hz
second publisher for actual speed or put in diagnostic?
do odom on mcu?
back right encoder is not working
use flash to save params
use mutext to lock status? https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#mutex
adc for battery
interpolators for counting pulses?