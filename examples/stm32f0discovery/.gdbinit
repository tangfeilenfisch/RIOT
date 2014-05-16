
file bin/stm32f0discovery/stm32f0test.elf

dir .
set prompt (cm0-gdb) 

# connect to openOCD running on gdb port 3333
target remote localhost:3333

set arm force-mode thumb
# Set a breakpoint
#b main
