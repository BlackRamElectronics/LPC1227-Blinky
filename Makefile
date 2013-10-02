include ./makefile.conf
NAME=LPC1227-Blinky
STARTUP_DEFS=-D__STARTUP_CLEAR_BSS -D__START=main

LDSCRIPTS=-L. -L$(BASE)/ldscripts -T gcc.ld
LFLAGS=$(USE_NANO) $(USE_NOHOST) $(LDSCRIPTS) $(GC) $(MAP)

all: $(NAME)-$(CORE).axf $(NAME)-$(CORE).bin program

$(NAME)-$(CORE).axf: $(NAME).c lpc1200/system_LPC12xx.c $(STARTUP)
	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@
	
$(NAME)-$(CORE).bin:$(NAME)-$(CORE).axf
	$(OBJCOPY) -O binary $(NAME)-$(CORE).axf $(NAME)-$(CORE).bin

program:$(NAME)-$(CORE).bin
	lpc21isp -wipe -bin -control $(NAME)-$(CORE).bin com1 115200 12000 
	
clean: 
	rm -f $(NAME)*.axf *.map $(NAME)*.bin
