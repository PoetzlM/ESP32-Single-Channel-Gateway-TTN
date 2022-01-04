#!/bin/bash
#rm -f sdkconfig
#rm -f sdkconfig.old

esptool.py --chip esp32 --port rfc2217://192.168.56.1:5555 erase_flash
