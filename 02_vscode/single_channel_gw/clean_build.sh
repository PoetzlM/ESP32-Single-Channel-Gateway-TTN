#!/bin/bash
#rm -f sdkconfig
#rm -f sdkconfig.old
rm -R build

idf.py fullclean
idf.py set-target esp32
idf.py build
