# Introduction #

Low-cost brushless motor controller for R/C (multi-rotor) use.
Fast response to PWM throttle, optional control by I2C (MikroKopter-compatible), or UART. Jitter-free ICP PWM input. Designed for SimonK firmware.

# Availability #

http://www.hobbyking.com/hobbyking/store/__869__710__Multi_Rotors_Parts-Afro_ESC_Series.html

# MikroKopter I2C assignment #

Default firmware ships for motor #1 only. User needs to build and flash firmware for other motor index.

https://github.com/sim-/tgy/blob/master/tgy.asm#L173

Set I2C\_ADDR to 0x50 (default motor start address).
Then change MOTOR\_ID to 1, 2, 3, 4,... etc to match motor index, rebuild firmware, and flash afro\_nfet.hex using USB linker.