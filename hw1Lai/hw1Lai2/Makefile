TARGET=hw1Lai

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        hw1Lai.c
	
OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
