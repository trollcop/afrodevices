= How to upload firmware using AfroFlash

## Standard method ##

  1. Use a servo cable to connect the FWUP socket to a USB->Serial converter.
  1. Pinout at the FC is RX TX GND, RX is right above Status LED.
  1. Power up the board (for example connect 1st ESC)
  1. Note COM port number of the USB serial converter
  1. Open command prompt, and go to the directory where flash utility and firmware is located.
  1. Firmware upgrade command is, "afroflashfc.exe <com port number> <firmware update file>"
  1. For example, "afroflashfc.exe 1 update.s19"


Output during successful update:

```
*** COM1 opened successfully.
*** Activating Bootloader: OK
*** Initializing Bootloader RAM... OK
*** UPDATING: [00, 0x8000, 1]... OK
*** UPDATING: [01, 0x8400, 1]... OK
... skipped
*** UPDATING: [10, 0xa800, 1]... OK
*** UPDATING: [11, 0xac00, 0]... OK
*** Rebooting...
*** OK OK OK Update Complete OK OK OK
*** Press any key to exit.
```

## Extended method ##

If the above method fails (firmware was too old), firmware needs to be flashed during bootloader timeout at power-up (0.5 seconds)


  1. Follow steps 1-5 in the 'standard method'
  1. Firmware upgrade command is, "afroflashfc.exe <com port number> <firmware update file> EX" (adding "ex" at the end from standard method)
  1. for example, "afroflashfc.exe 1 update.s19 ex"
  1. Get ready to press enter on the command line above, and power up the board.
  1. Power up the board and enter the command line within .5 sec of power up.
  1. Firmware should now continue updating