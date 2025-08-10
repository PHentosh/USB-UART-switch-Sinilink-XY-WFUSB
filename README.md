## UART USB-Switch

This project is a simple USB-switch based on [Sinilink XY-WFUSB USB Switch Relay](https://devices.esphome.io/devices/Sinilink-XY-WFUSB-USB-Switch-Relay).
But instead of wifi it uses UART to toggle the switch.

### Usage

Simply connect to the UART and sent `0` or `1` to turn `ON` of `OFF` the switch.

```
0
I (8422) USB-SW: OFF
1
I (8544) USB-SW: ON
```

You can also use a button to toggle the switch.

### Build and Flash

This project is written for `esp8265` chip as it's installed on Sinilink XY-WFUSB.
So to build this code you will have so setup an esp toolchain and SDK:

- [Toolchain](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/index.html)
- [ESP8266-RTOS-SDK](https://github.com/espressif/ESP8266_RTOS_SDK)

**Note:** Unfortunately those tools are still using Python 2.7, so you cna make a virtual environment for easier development

**Note:** There are some known bugs in those tools so you will have to make some changes for succesfull instalation:

- In `ESP8266-RTOS-SDK/tools/kconfig/lxdialog/check-lxdialog.sh` you sould find line 66 and make the following fix
```
-main() {}
+int main() {}
```

- In `ESP8266-RTOS-SDK/requirements.txt` you sould set `pyelftools` version **strictly** to `0.22`. You may encounter some other simmilar issues due to evolving python packages.
```
-pyelftools>=0.22
+pyelftools==0.22
```

After you installed all of the above, some configurations in esp-idf should be made.
You can access them with a following command:
```
$> make menuconfig
```

What you should change:
- "Serial flash config" -> "Flash SPI mode" = "DOUT"
- "Serial flash config" -> "Flash size" = "1Mb"
- "Component config" -> "ESP8266-specific" -> "CPU frequency" = "80Mhz"

Also you can change the UART frequency for your convenience:
- "Component config" -> "Common ESP-related" -> "UART console baud rate"
- "Serial flash config" -> "make monitor baud rate"

With those toolse configured, we can build and flash the app with only one command
```
$> make flash
```
You may need to also specify the port.