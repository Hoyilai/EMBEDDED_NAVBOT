TARGET=hw9Lai

SOURCES=import_registers.c \
	raspicam_wrapper.cpp \
	enable_pwm_clock.c \
	hw9Lai.c \
	scale_image_data.c 

OBJECTS=$(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SOURCES)))

CFLAGS=`pkg-config --cflags --libs gtk+-2.0` \
       -Wall \
       -Werror

all: $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	g++ $^ $(CFLAGS) -o $@ -lraspicam -lm -lpthread

%.o:%.c
	gcc $(CFLAGS) -c $< -o $@

%.o:%.cpp
	g++ $(CFLAGS) -c $< -o $@
