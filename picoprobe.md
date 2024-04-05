# Picoprobe
Debugger. Can use another pico flashed with the UF2.

https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf

https://www.digikey.com/en/maker/projects/raspberry-pi-pico-and-rp2040-cc-part-2-debugging-with-vs-code/470abc7efb07432b82c95f6f67f184c0

https://blog.smittytone.net/2021/02/05/how-to-debug-a-raspberry-pi-pico-with-a-mac-swd/


```sh
brew install cmake
brew tap ArmMbed/homebrew-formulae
brew install gcc-arm-embedded
brew install libtool automake libusb wget pkg-config gcc texinfo

cd ~/Downloads

cd ~/pico
git clone https://github.com/raspberrypi/openocd.git --branch rp2040-v0.12.0 --depth=1 
cd openocd
export PATH="/usr/local/opt/texinfo/bin:$PATH" 1
./bootstrap
./configure --disable-werror 2
make -j4
```

.vscode/settings.json
```json
{
    "cmake.environment": {
        "PICO_SDK_PATH":"${env:PICO_SDK_PATH}"
    },
     "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools"
}
```

## Wiring
```
Pico A GND -> Pico B GND
Pico A GP2 -> Pico B SWCLK
Pico A GP3 -> Pico B SWDIO
Pico A GP4/UART1 TX -> Pico B GP1/UART0 RX
Pico A GP5/UART1 RX -> Pico B GP0/UART0 TX
```

If Pico B is a USB Host then you must connect VBUS to VBUS, not VSYS to VSYS, so that Pico B can provide 5V on its USB connector. If Pico B is using USB in device mode, or not using its USB at all, this is not necessary.