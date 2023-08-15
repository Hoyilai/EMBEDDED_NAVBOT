TARGET=hw7Lai

SOURCES=import_registers.c \
        enable_pwm_clock.c \
	wait_key.c \
        hw7Lai.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -lpthread -o $(TARGET) -lm

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
