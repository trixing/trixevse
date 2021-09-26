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
-  modbus interface
-  HTTP JSON API
   ```
   http://trixing-evse.local/
   ```

## Updating

Location of bin file on macOS:
```
find /var/folders/ -name '*.bin'
```

Run update via curl
```
curl -F "image=@trixing-evse-wifi.ino.bin" http://trixing-evse.local/update
```

## Parts

-  EVSE DIN w/ RS485
   https://www.evse-wifi.de/produkt/evse-din-ladecontroller-rs485/
