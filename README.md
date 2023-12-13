# Bare metal scheduler for stm32f4gxx microcontroller family

## Introduction
The project consists of a round robin scheduler running on the ARM Cortex-M4 processor of a STM32F407 target.

For this use case, 5 user tasks have been used:
- task1 with a periodicity of T. When scehduled it toggles the gpio pin connected to the green led
- task2 with a periodicity of T/2. When scehduled it toggles the gpio pin connected to the orange led 
- task3 with a periodicity of T/4. When scehduled it toggles the gpio pin connected to the red led 
- task4 with a periodicity of T/8. When scehduled it toggles the gpio pin connected to the blue led 
- idle_task to fill empty slices


## Setting up on a stm32f407xxx
Clone the repository and build the prioject:
```bash
git clone https://github.com/nicolamusacchio99/rr_scheduler_stm32f407.git
cd rr_scheduler_stm32f407
make all
```

The executable (.elf) will be stored in the bin/ folder

## Load the executable
First install OpenOCD and gdb (arm-gnu-toolchain). 
OpenOCD will act as server application on the host machine. A gdb client can then connect to the OpenOCD server and allows the user to send data to the JTAG debugger of the embedded target.

Open the terminal and start the OpenOCD server with the proper configuration file board/stm32f4discovery.cfg
```bash
make load
```
openocd -f board/stm32f4discovery.cfg to make server listening on port 3333


Open a gdb client in another terminal and connect to the OpenOCD server:
```bash
arm-none-eabi-gdb
target remote localhost:3333
monitor reset init
```

Engrave executable and run the scheduler:
```bash
monitor flash write_image erase scheduler.elf
monitor reset halt
mointor continue
```

