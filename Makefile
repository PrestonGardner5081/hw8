TARGET1=hw8gardner
TARGET2=hw5Calibration

SOURCES1=import_registers.c \
        enable_pwm_clock.c \
		blue_stack.c \
		float_queue.c \
		pwm_init.c \
		6050Init.c \
		hw8gardner.c

SOURCES2=import_registers.c \
        enable_pwm_clock.c \
		blue_stack.c \
		float_queue.c \
		pwm_init.c \
		6050Init.c \
		hw5Calibration.c 
 
OBJECTS1=$(patsubst %.c,%.o,$(SOURCES1))

OBJECTS2=$(patsubst %.c,%.o,$(SOURCES2))


all: hw8gardner 

hw8gardner: $(OBJECTS1)
	gcc $(OBJECTS1) -o $(TARGET1) -lpthread -lm

hw5Calibration: $(OBJECTS2)
	gcc $(OBJECTS2) -o $(TARGET2) -lpthread -lm

clean:
	rm -f $(OBJECTS1) $(TARGET1)
	rm -f $(OBJECTS2) $(TARGET2)

%.o:%.c
	gcc -o2 -c $< -o $@
