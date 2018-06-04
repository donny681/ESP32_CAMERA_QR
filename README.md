# ESP32 Camera Demo

Code provided in this repository gets the image from camera and prints it out as ASCII art to the serial port.

## Function
+ support OV2640 to output JPEG format .
+ support OV2640 and OV7725 to output grayscale format.
+ support QR recognize.

## Components

To make this code work, you need the following components:

* This repository. It contains submodules, so make sure you clone it with `--recursive` option. If you have already cloned it without `--recursive`, run `git submodule update --init`.
* [ESP32](https://espressif.com/en/products/hardware/esp32/overview) module
* Camera module
* PC with [esp-idf](https://github.com/espressif/esp-idf)

See the following sections for more details.


### Camera

This example has been tested with OV7725 and ov2640 camera modules. Use it, if this is your first exposure to interfacing a microcontroller with a camera.

Other OV7xxx series should work as well, with some changes to camera configuration code. OV5xxx can work too, but it is advisable to choose the ones which support RGB or YUV 8-bit wide output bus. The ones which only output 10-bit raw data may be a bit harder to work with. Also choose the camera which can output a scaled down (QVGA or VGA) image. Use of larger frame buffers will require external SPI RAM.

### Ai-Thinker CAM board

Ai-Thinker Launch a camera board.This board contain camera interface,TF card and ESP32 with psram.It is very a samll module.The size of the board is 40mm * 26mm * 4mm.You can plug the camera in the board.We will sell this board in our [taobao](https://shop72165205.taobao.com/?spm=a230r.7195193.1997079397.26.Z8Ck3C).

![Aithinker camera board](https://github.com/donny681/ESP32_CAMERA_QR/blob/master/pictures/back.jpg)
![Aithinker camera board front](https://github.com/donny681/ESP32_CAMERA_QR/blob/master/pictures/front.jpg)
![Aithinker camera board qr recognize](https://github.com/donny681/ESP32_CAMERA_QR/blob/master/pictures/aithinker_camer_board%20qr%20recognize.jpg)
Also,if you have an ESP-WROVER-KIT,you can try this demo.

## Quick Start

If you have your components ready, follow this section to [connect](#connect) the camera to ESP32 module, [flash](#flash) application to the ESP32 and finally [shoot](#shoot) and display the image.

## Connect

Specific pins used in this example to connect ESP32 and camera are shown in table below. Pinout can be adjusted to some extent in software. Table below provides two options of pin mapping (last two columns).

| Interface | Camera Pin | Pin Mapping for ESP32 DevKitJ | Ai-Thinker CAM board |
| :--- | :---: | :---: | :---: |
| SCCB Clock | SIOC | IO27 | IO27 |
| SCCB Data | SIOD | IO26 | IO26 |
| System Clock | XCLK | IO21 | IO0 |
| Vertical Sync | VSYNC | IO25 | IO25 |
| Horizontal Reference | HREF | IO23 | IO26 |
| Pixel Clock | PCLK | IO22 | IO0 |
| Pixel Data Bit 0 | D2 | IO4 | IO5 |
| Pixel Data Bit 1 | D3 | IO5 | IO18 |
| Pixel Data Bit 2 | D4 | IO18 | IO19 |
| Pixel Data Bit 3 | D5 | IO19 | IO21 |
| Pixel Data Bit 4 | D6 | IO36 | IO36 |
| Pixel Data Bit 5 | D7 | IO39 | IO39 |
| Pixel Data Bit 6 | D8 | IO34 | IO34 |
| Pixel Data Bit 7 | D9 | IO35 | IO35 |
| Camera Reset | RESET | IO2 | 3.3V |
| Camera Power Down | PWDN | *see Note 1* | GND |
| Power Supply 3.3V | 3V3 | 3V3 | IO32 |
| Ground | GND | GND | GND |

Notes:

1. **Camera pin** column refers to pinout on OV7725 camera module
2. **Camera Power Down** pin does not need to be connected to ESP32 GPIO. Instead it may be pulled down to ground with 10 kOhm resistor.

### Flash

Clone the code provided in this repository to your PC, compile with the latest [esp-idf](https://github.com/espressif/esp-idf) installed from GitHub and download to the module.

If all h/w components are connected properly you are likely to see the following message during download:

```
Krzysztof@tdk-kmb-op780 MSYS /esp/esp32-cam-demo
$ make flash
Flashing binaries to serial port com18 (app at offset 0x10000)...
esptool.py v2.0-dev
Connecting...

A fatal error occurred: Failed to connect to ESP32: Timed out waiting for packet header
make: *** [C:/msys32/esp-idf/components/esptool_py/Makefile.projbuild:48: flash] Error 2
```
This is due to a pullup on the camera reset line. It is stronger than the internal pull-down on `GPIO2` of the ESP32, so the chip cannot go into programming mode.

There are couple of options how to resolve this issue:

* If you are using ESP-WROVER V1 then connect GPIO2 to GND while flashing.
* Power down the camera module by removing it from the socket (ESP-WROVER V1) or by uplugging 3.3V wire.
* Map Camera Reset line to another GPIO pin on ESP32, for instance `GPIO15`.

### Shoot

Once module is loaded with code, open a serial terminal.

Camera demo application will first configure XCLK output that is timing operation of the camera chip.

If you set the pin of the xclk as GPIO0,the clock will be output by I2S1.The wave of clock is the best.
```
I (71) I2S: DMA Malloc info, datalen=blocksize=256, dma_buf_count=8
I (71) I2S: PLL_D2: Req RATE: 78125, real rate: 78125.000, BITS: 16, CLKM: 8, BCK: 8, MCLK: 20000000.000, SCLK: 2500000.000000, diva: 64, divb: 0
I (81) camera_xclk: PIN_CTRL before:3ff
I (81) camera_xclk: PIN_CTRL after:7fff
```
If you set the pin of the xclk as other GPIO,the clock will be output by ledc.
```
D (1527) camera: Enabling XCLK output
I (1527) ledc: LEDC_PWM CHANNEL 0|GPIO 21|Duty 0004|Time 0
```
This clock is also timing output of pixel data on camera output interface - see I2S and DMA described below.

Then [SCCB](http://www.ovt.com/download_document.php?type=document&DID=63) interface is set up:

```
D (1527) camera: Initializing SSCB
I (1537) gpio: GPIO[26]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1537) gpio: GPIO[27]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1547) gpio: GPIO[26]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (1557) gpio: GPIO[27]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
```

In next step the communication with camera should be established. ESP will retrieve camera's address and signature.

```
D (1567) camera: Resetting camera
D (1587) camera: Searching for camera address
D (1587) camera: Detected camera at address=0x21
D (1587) camera: Camera PID=0x77 VER=0x21 MIDL=0x7f MIDH=0xa2
```

If communication fails, the following message is shown:

```
E (1076) camera: Camera address not found
E (1076) camera_demo: Camera init failed with error = 131073
```

If communication with camera module is established, ESP will reset the camera sensor and reserve memory for video frame buffer:

```
D (1587) camera: Doing SW reset of sensor
D (1647) camera: Setting frame size at 320x240
D (1677) camera: Allocating frame buffer (320x240, 76800 bytes)
```

Image from camera is retrieved using I2S communication for all eight pixel bits at once and saved in memory line by line. Log below shows completion of initialization steps for I2S and DMA:

```
D (1677) camera: Initializing I2S and DMA
I (1677) gpio: GPIO[35]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1677) gpio: GPIO[34]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1687) gpio: GPIO[39]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1697) gpio: GPIO[36]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1707) gpio: GPIO[19]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1717) gpio: GPIO[18]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1727) gpio: GPIO[5]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1737) gpio: GPIO[4]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1747) gpio: GPIO[25]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1757) gpio: GPIO[23]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1767) gpio: GPIO[22]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
D (1777) intr_alloc: Connected src 32 to int 3 (cpu 0)
D (1777) camera: Allocating DMA buffer #0, size=1280
D (1787) camera: Allocating DMA buffer #1, size=1280
D (1817) camera: Init done
```



## How it Works

### Software

The core of camera software is contained in `camera` folder and consists of the following files.

* [camera.c](components/camera/camera.c) and [include/camera.h](components/camera/include/camera.h) - main file responsible for configuration of ESP32's GPIO, clock, I2S and DMA to interface with camera module. Once interface is established, it perfroms camera configuration to then retrieve image and save it in ESP32 memory. Access to camera is executed using lower level routines in the following files.

* [ov7725.c](components/camera/ov7725.c), [ov7725.h](components/camera/ov7725.h), [ov7725_regs.h](components/camera/ov7725_regs.h) and [sensor.h](components/camera/sensor.h) - definition of registers of OV7725 to configure camera funcinality. Functions to set register groups to reset camera to default configuration and configure specific functionality like resolution or pixel format. Setting he registers is performed by lower level function in files below.

* [sccb.c](components/camera/sccb.c) and [sccb.h](components/camera/sccb.h) - implementation of [Serial Camera Control Bus (SCCB)](http://www.ovt.com/download_document.php?type=document&DID=63) protocol to set camera registers.

* [twi.c](components/camera/twi.c) and [twi.h](components/camera/twi.h) - implementation of software I2C routines used by SCCB protocol.

* [wiring.c](components/camera/wiring.c) and [wiring.h](components/camera/wiring.h) - the lowest level routines to set GPIO pin mode, set GPIO pin level and delay program execution by required number of ms.

* [component.mk](components/camera/component.mk) - file used by C `make` command to access component during compilation.

* [Kconfig.projbuild](components/camera/Kconfig.projbuild) - file used by `make menuconfig` that provides menu option to switch camera test pattern on / off.

All above are called _esp-idf component_ and placed in `components` folder. Esp-idf framework provides `components` folder as a standard place to add modular functionality to a project.

Application starts and the top level control is executed from [app_main.c](main/app_main.c) file located in [main](main) folder.


## Troubleshooting

If you have issues to get the live image right, enable test pattern and see what is retrieved.

To do so, run `make menuconfig`, open `Example Configuration` menu option and check `[ ] Enable test pattern on camera output`.

Optionally change the following define in file `camera.c`:

```
# define ENABLE_TEST_PATTERN CONFIG_ENABLE_TEST_PATTERN
```

Camera sensor will then output test pattern instead of live image.

```
D (5692) camera: Waiting for positive edge on VSYNC
D (5722) camera: Got VSYNC
D (5722) camera: Waiting for frame
D (5752) camera: Frame done
D (5752) camera_demo: Done
|@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%########## +++++++++==========-:::::::::          |
|@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%########## +++++++++==========-:::::::::          |
|@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%########## +++++++++==========-:::::::::          |
|@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%########## +++++++++==========-:::::::::          |
|@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%########## +++++++++==========-:::::::::          |
|@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%########## +++++++++==========-:::::::::          |
|@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%########## +++++++++==========-:::::::::          |
|@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%########## +++++++++==========-:::::::::          |
```

O
nce test pattern is enabled, application will calculate standard variance of what is retrieved by ESP32 against pattern generated by the camera module. If there is noise on the lines or some pixels lines not connected or shortcut, this should be reported below displayed image. 

```
Frames / mismatch : 4630 / 70 (1%), (6531449)
```
The value in brackets with % sign provides number of frames that differ from the test pattern. See code inline comments for more information on this functionality.

## QR Recognition
ESP32_CAMERA_QR contain QR library.
To do so, run `make menuconfig`, open `ESP32 Camera Demo Configuration` menu option and check QR recognize Support.
Then change the following define in file `app_main.c`:
```
#define CAMERA_PIXEL_FORMAT CAMERA_PF_GRAYSCALE
#define CAMERA_FRAME_SIZE CAMERA_FS_QVGA
```
note: 
+ If you want to recognize QR,the CAMERA_PIXEL_FORMAT must be seted as CAMERA_PF_GRAYSCALE.
+ The size of photo can't be over VGA.Or the ram may be  oversize.

![QR recognition](https://github.com/donny681/ESP32_CAMERA_QR/blob/master/pictures/qr_recognize.jpg)

## Next Steps

We are planning to test and compare images captured using:

+ Planning to transplant [openmv](https://github.com/openmv/openmv)
+ Planing to increase the framerate.

## Help
+ Hope developers to provide the driver of OV2640 cif mode.
+ We are going to add other camera.if you have the driver of new camera,please tell to me. 



