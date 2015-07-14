rev1 fixed some design omissions in rev0, but was never manufactured.
rev2 accepted the OpenPilot PCB dimensions of 36x36mm with 30.5mm mounting holes. This will be the final production version.

Slightly larger board size allowed to place label silkscreen on top side of the board only, easing user assembly and testing.

# Connections #

## Board Top and Bottom ##
![http://i.imgur.com/TCQnJ.png](http://i.imgur.com/TCQnJ.png)

All needed connectors are at the top of the board.

  * Motors 1-6
  * PPM for RC input
  * UART for telemetry or configuration
  * BUZ for external 5V buzzer (used for battery low alarm)
  * I2C header for additional sensor/extension boards
  * VBAT connector for battery low alarm
  * Power and Status LEDs