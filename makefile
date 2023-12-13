CC=arm-none-eabi-gcc
MACH=cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall
INCLUDES = -I inc
SRCDIR = src
OBJDIR = obj
BINDIR = bin
LDFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T stm32_ls.ld -Wl,-Map=scheduler.map
LDFLAGS_SH = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=rdimon.specs -T stm32_ls.ld -Wl,-Map=scheduler.map

all:$(OBJDIR)/main.o $(OBJDIR)/led.o $(OBJDIR)/stm32_startup.o $(OBJDIR)/scheduler.o $(OBJDIR)/init_scheduler_conf.o $(OBJDIR)/user_tasks.o $(OBJDIR)/gpio_driver.o $(OBJDIR)/syscalls.o $(BINDIR)/scheduler.elf
semi:main.o led.o stm32_startup.o final_sh.elf 

$(OBJDIR)/main.o:$(SRCDIR)/main.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@  $^

$(OBJDIR)/led.o:$(SRCDIR)/led.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@  $^

$(OBJDIR)/init_scheduler_conf.o:$(SRCDIR)/init_scheduler_conf.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@  $^

$(OBJDIR)/scheduler.o:$(SRCDIR)/scheduler.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@  $^

$(OBJDIR)/user_tasks.o:$(SRCDIR)/user_tasks.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@  $^

$(OBJDIR)/gpio_driver.o:$(SRCDIR)/gpio_driver.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@  $^

$(OBJDIR)/stm32_startup.o:startup/stm32_startup.c
	$(CC) $(CFLAGS) -o $@  $^

$(OBJDIR)/syscalls.o:$(SRCDIR)/syscalls.c
	$(CC) $(CFLAGS) -o $@  $^

$(BINDIR)/scheduler.elf: $(OBJDIR)/main.o $(OBJDIR)/led.o $(OBJDIR)/stm32_startup.o $(OBJDIR)/init_scheduler_conf.o $(OBJDIR)/scheduler.o $(OBJDIR)/user_tasks.o $(OBJDIR)/gpio_driver.o $(OBJDIR)/syscalls.o
	$(CC) $(LDFLAGS) -o $@  $^

final_sh.elf: main.o led.o nit_scheduler_conf.o scheduler.o user_tasks.o stm32_startup.o
	$(CC) $(LDFLAGS_SH) -o $@  $^

clean:
	rm -rf $(OBJDIR)/*.o $(BINDIR)/*.elf *.map

load:
	openocd -f board/stm32f4discovery.cfg