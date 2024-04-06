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
ros2 topic pub --once deepdrive_micro/cmd std_msgs/msg/Int32 "{data: 5000}"
ros2 topic pub --once deepdrive_micro/cmd std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once deepdrive_micro/cmd std_msgs/msg/Int32 "{data: -5000}"
```

TODO: 1000 stops
2000 is slow
3000 is a good pace
