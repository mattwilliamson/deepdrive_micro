# deepdrive_micro

ros micro interface to raspberry pi pico

Interfaces:
- LEDs
- Sonar
- Buzzer
- Battery Voltage

## Installation

```sh
sudo apt update && sudo apt install libhidapi-hidraw0 libhidapi-libusb0
sudo apt install automake autoconf build-essential texinfo libtool libhidapi-dev libusb-1.0-0-dev gdb-multiarch

cd ~/.platformio/packages/tool-openocd-rp2040-earlephilhower/share/openocd/scripts/
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 1000" -c "program $HOME/src/deepdrive_motor_controller/.pio/build/esp32dev/firmware.elf verify reset exit"
Error: Failed to connect multidrop rp2040.dap0
```


### Option1: Setup OpenOCD / Raspbery Pi Debug Probe

Buy a debug probe from Raspberry Pi foundation or flash another pico with the firmware to act like one.

https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html
https://github.com/raspberrypi/debugprobe

#### Udev rules

https://docs.platformio.org/en/latest/core/installation/udev-rules.html#platformio-udev-rules

```sh
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
curl -fsSL https://raw.githubusercontent.com/raspberrypi/picotool/master/udev/99-picotool.rules | sudo tee /etc/udev/rules.d/99-picotool.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

```sh
$ pio device list
/dev/ttyACM0
------------
Hardware ID: USB VID:PID=2E8A:000A SER=454741505A85844A LOCATION=1-2.4.2:1.0
Description: Pico - Board CDC

/dev/ttyACM1
------------
Hardware ID: USB VID:PID=2E8A:000C SER=E6626005A7907534 LOCATION=1-2.3:1.1
Description: Debugprobe on Pico (CMSIS-DAP) - CDC-ACM UART Interface
```

Plug in picoprobe.

```sh
$ sudo dmesg
[91616.853815] usb 1-2.1: new full-speed USB device number 17 using tegra-xusb
[91616.969400] usb 1-2.1: New USB device found, idVendor=2e8a, idProduct=000c, bcdDevice= 2.00
[91616.969410] usb 1-2.1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[91616.969415] usb 1-2.1: Product: Debugprobe on Pico (CMSIS-DAP)
[91616.969419] usb 1-2.1: Manufacturer: Raspberry Pi
[91616.969422] usb 1-2.1: SerialNumber: E6626005A7907534
[91616.975556] cdc_acm 1-2.1:1.1: ttyACM1: USB ACM device
```

```sh
sudo apt install libudev-dev

cat << EOF | sudo tee /etc/udev/rules.d/61-openocd.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000c", MODE="660", GROUP="plugdev", TAG+="uaccess", SYMLINK+="picoprobe"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

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
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="660", GROUP="plugdev", SYMLINK+="deepdrive_micro"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify

```sh
$ ls -lah /dev/deepdrive_micro
lrwxrwxrwx 1 root root 15 Jul 29 07:37 /dev/deepdrive_micro -> bus/usb/001/024
```


### Option 2: Setup picotool

Update platformio.ini with :

```ini
upload_protocol = picotool
```

Plug in to usb while holding `boot_sel` button.

```sh
$ sudo dmesg -w
[171216.880875] usb 1-2.4.2: new full-speed USB device number 52 using tegra-xusb
[171216.993538] usb 1-2.4.2: New USB device found, idVendor=2e8a, idProduct=0003, bcdDevice= 1.00
[171216.993549] usb 1-2.4.2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[171216.993556] usb 1-2.4.2: Product: RP2 Boot
[171216.993563] usb 1-2.4.2: Manufacturer: Raspberry Pi
[171216.993568] usb 1-2.4.2: SerialNumber: E0C9125B0D9B
[171216.995770] usb-storage 1-2.4.2:1.0: USB Mass Storage device detected
[171216.996946] scsi host0: usb-storage 1-2.4.2:1.0
[171218.014872] scsi 0:0:0:0: Direct-Access     RPI      RP2              3    PQ: 0 ANSI: 2
[171218.024093] sd 0:0:0:0: [sda] 262144 512-byte logical blocks: (134 MB/128 MiB)
[171218.032936] sd 0:0:0:0: [sda] Write Protect is off
[171218.037975] sd 0:0:0:0: [sda] Mode Sense: 03 00 00 00
[171218.039218] sd 0:0:0:0: [sda] No Caching mode page found
[171218.044788] sd 0:0:0:0: [sda] Assuming drive cache: write through
[171218.088474]  sda: sda1
[171218.092332] sd 0:0:0:0: [sda] Attached SCSI removable disk
^C

$ blkid
/dev/sda1: SEC_TYPE="msdos" LABEL_FATBOOT="RPI-RP2" LABEL="RPI-RP2" UUID="000A-052D" TYPE="vfat" PARTUUID="000a052d-01"

```

```sh
sudo mkdir /mnt/deepdrive_micro
sudo vi /etc/fstab

# /etc/fstab Raspberry Pi Pico
LABEL="RPI-RP2" /mnt/deepdrive_micro vfat auto,user,rw,exec 0 0

sudo mount /mnt/deepdrive_micro
```

```sh
$ df -h /mnt/deepdrive_micro/
Filesystem      Size  Used Avail Use% Mounted on
/dev/sda1       128M  8.0K  128M   1% /mnt/deepdrive_micro

$ mount -l | grep deepdrive
/dev/sda1 on /mnt/deepdrive_micro type vfat (rw,nosuid,nodev,relatime,fmask=0022,dmask=0022,codepage=437,iocharset=iso8859-1,shortname=mixed,errors=remount-ro,user)
/dev/sdb1 on /mnt/deepdrive_micro type vfat (rw,nosuid,nodev,relatime,uid=1000,gid=1000,fmask=0002,dmask=0002,allow_utime=0020,codepage=437,iocharset=iso8859-1,shortname=mixed,errors=remount-ro,user=matt) [RPI-RP2]

$ picotool info -a
Program Information
 name:          deepdrive_micro
 features:      USB stdin / stdout
 binary start:  0x10000000
 binary end:    0x1003c974

Fixed Pin Information
 none

Build Information
 sdk version:       1.5.1
 pico_board:        pico
 boot2_name:        boot2_w25q080
 build date:        May 10 2024
 build attributes:  Debug

Device Information
 flash size:   2048K
 ROM version:  3

```

### Option 3: mbed (untested)

Same as option 2 but you need to specify upload location in `platformio.ini`

```ini
upload_protocol = mbed
upload_port = /mnt/deepdrive_micro
```




## Run micro ros Agent
```sh
mamba activate ros_env
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

ros2 topic echo deepdrive_micro/pulses
```



---

## Notes

Arduino-pico is based on Mbed. Mbed is not multicore aware, so anything running on core1 can't be FreeRTOS tasks. It must be pure Pico-SDK code.

[More info](https://github.com/arduino/ArduinoCore-mbed/issues/242)

