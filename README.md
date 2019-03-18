# Esk8Monitor
This project hosts the Arduino files that are used to monitor some values that are provided by the VESC motor controllers in my electric skateboard
## Features
There are two components:
- Generic ESP32 dev board (the "Monitor") that is on the skateboard, communicating to the VESC (master) through UART0
- a M5Stick (the "MonitorStick") that has a OLED display that will show the stats
The Monitor talks/communicates with the MonitorStick using BLE.
## TODO
- if the client (MonitorStick) cannot connect to the server (Monitor) then it hangs and you can't shut the MonitorStick down.
  - solution: if you reboot while holding the other button down, you can deepsleep the MonitorStick before checking for the server
