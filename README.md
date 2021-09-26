# TrixEVSE - A custom electrical vehicle charging station

If your electrical vehicle charging station is broken and
the replacement part gets quoted north of the price of a new
wallbox this project might be for you.

This is taking a WallBe Eco 2.0s as a base, but most likely
works with a lot of other boxes which are built from off
the shelf parts and a Phoenix charge controller.

The documentation is work in progress.

## Features

-  mDNS listener at trixing-evse.local
-  Modbus interface (emulating an AC22 charger from Victron)
   See `https://github.com/victronenergy/dbus-modbus-client/blob/master/ev_charger.py`
-  HTTP JSON API `http://trixing-evse.local/j`

## Updating

Location of bin file on macOS:
```
find /var/folders/ -name '*.bin'
```

Run update via curl
```
curl -F "image=@trixing-evse-wifi.ino.bin" http://trixing-evse.local/update
```

## Software

-  Arduino 1.8.13
-  ArduinoJSON library
-  ModbusIP library
-  U8x8 library

## Parts

-  Broken Wallbox (I used a Wallbe Eco 2.0s)
-  Wemos D1 R1 Mini V3 ESP8266 based WiFi microcontroller
-  RS48 TTL converter based on MAX485
-  OLED I2C Display
   Makerhawk I2C SSD1306 128x32 3.3V/5V
-  Push Button
-  SDM120 Modbus Din Rail power meter
-  Din Rail Normally Open 2-phase relay
   e.g. Heschen CT1-25 2-pole Normally Open, 25A, 230V
-  EVSE DIN w/ RS485
   https://www.evse-wifi.de/produkt/evse-din-ladecontroller-rs485/
