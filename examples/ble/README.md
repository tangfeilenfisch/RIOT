## Initial nRF51822 support and youngish Blueooth LE Stack for RIOT
There are link layer manager and controller interface, sys/ble/blell.c and sys/ble/bleci.cc.
Phy driver (drivers/nrf51822.c) supports slave mode only (who needs a sensor acts as master :-)).
Link layer and controller interface are incomplete and less commented.

## nRF51822 Module
I have got my module from ebay seller chipworld for about 5 Euro. The module have a PCB-Antenne and
16MHz oscillator on board. All pins are accessibly over 2 x 18 pin header (2mm raster).
For starting you need a power source and swd adapter for openocd.
Actually i use stm32f0discovery board as swd adapter and power source. That was the fastest and
cheapest way for me to start with nRF51822. Also you need the nrf51-refmanual and PAN from
Nordic-Semiconductor. As debian user i don't worry about toolchain.

## ble example
A UART adapter can be connected on the pins GPIO3 and GPIO4 (see boards/nrf51-proto).
Bluetooth LE example (examples ble) starts link layer in advertising mode for 60 sec.
You can scan or connect the module using hcitool like:

hcitool -i hci1 lescan
hcitool -i hci1 lecc --random 42:42:42:42:42:42

You may find btmon tool very helpful too.

### start openocd
openocd -f board/stm32f0discovery.cfg
