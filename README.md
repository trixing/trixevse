
## Features

-  mDNS listener at trixing-evse.local
-  modbus interface

## Updating

Location of bin file on macOS:
```
find /var/folders/ -name '*.bin'
```

Run update via curl
```
curl -F "image=@trixing-evse-wifi.ino.bin" trixing-evse.local/update
```

## Parts

-  EVSE DIN w/ RS485
   https://www.evse-wifi.de/produkt/evse-din-ladecontroller-rs485/


