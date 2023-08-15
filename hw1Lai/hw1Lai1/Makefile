TARGET=hw1Lai1

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        hw1Lai1.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
