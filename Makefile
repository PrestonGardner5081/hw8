TARGET1=hw8gardner

SOURCES1=import_registers.c \
        enable_pwm_clock.c \
		blue_stack.c \
		float_queue.c \
		pwm_init.c \
		6050Init.c \
		malloc_image.c \
		process.c \
		raspicam_wrapper.cpp \
		hw8gardner.c
 
OBJECTS1=$(patsubst %.c,%.o,$(SOURCES1))
OBJECTS2=$(patsubst %.cpp,%.o,$(OBJECTS1))

all: hw8gardner 

hw8gardner: $(OBJECTS1)
	g++ $(OBJECTS1) -o $(TARGET1) -lpthread -lraspicam

clean:
	rm -f $(OBJECTS2) $(TARGET1)

%.o:%.c
	gcc -o3 -c $< -o $@

%.o:%.cpp
	g++ -c $< -o $@