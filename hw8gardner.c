/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 5
* On 02/19/2022
* By Preston Gardner
* 
***************************************************/

/* Homework 5 
 * User input from terminal controls robot: 
 * (1) Stop: ' s '
 * (2) Forward: ' w '
 * (3) Backward: ' x '
 * (4) Faster: ' i ' 5% PWM power increase for each ‘i’ key hit
 * (5) Slower: ' j ' 5% PWM power decrease for each ‘j’ key hit
 * (6) Left: ' a ' 15 degree turn for each ‘a’ key hit, smooth transition
 * (7) Right: ' d ' 15 degree turn for each ‘d’ key hit, smooth transition
 * (8) Quit: ' q ' to quit all program proper (without cnt’l c, and without an Enter key)
 * Raspberry Pi 3 computer with
 * Motor driver controlled by  
 * PWM on GPIO12 -> PWMA (left motors) and GPIO13 -> PWMB (right motors)
 * GPIO05 -> AI1, GPIO06 -> AI2 (left motor direction control) 
 * GPIO22 -> BI1, GPIO23 -> BI2 (right motor direction control) 
 * 
 * IMPORTANT! This project is made for the raspberry pi 3. The base memory 
 * address in import_registers.c needs to be 0xFE000000 to run on rasp pi 4
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "blue_stack.h"
#include "pwm_init.h"
#include "6050Init.h"
#include "float_queue.h"
#include "raspicam_wrapper.h"
#include "raspicam_wrapper.h"
#include "process.h"

#define PI 3.14159265358979
#define IMG_RED_HEIGHT 260 / 2
#define IMG_RED_WIDTH 320

const int PWM_MAX = 1000;
const int PWM_MIN = 260;
const int PWM_RANGE = PWM_MAX - PWM_MIN;
const int PWM_50_PERC = PWM_MIN + PWM_RANGE / 2;
const int PWM_5_PERC = PWM_RANGE / 20;
const int RIGHT_MAX = 926;
const int TURN_LOW = PWM_MIN + 2 * PWM_5_PERC;
const int RAMP_TIME = 15; //us interval to ramp up / down speed
const int INSTR_QUEUE_SIZE = 1024;
const int DATA_QUEUE_SIZE = 10000; // about 1.67 minutes of data
const long TURN_TIME = 300000;
const long Z_C_TURN_TIME = TURN_TIME / 2;
const long CHECK_IR_INT = TURN_TIME / 100;
const long THREAD_PROCESS_TIME_uS = 10000;      //10000 uS = 10 ms
const long THREAD_PIC_PROCESS_TIME_uS = 100000; //100000 uS = 100 ms
const long CLOCKS_PER_MILLI = CLOCKS_PER_SEC / 1000;
const long CLOCKS_PER_MIRCO = CLOCKS_PER_MILLI / 1000;
const float ACC_PRECISION = 0.001;

int leftLookup[1000];

float xVelocity = 0, yVelocity = 0, heading = 0; // m/s
float xPos = 0, yPos = 0;                        // m

struct io_peripherals *io;
struct calibration_data calibration_accelerometer;
struct calibration_data calibration_gyroscope;
struct calibration_data calibration_magnetometer;
struct raspicam_wrapper_handle *Camera;

pthread_mutex_t inputLock, leftControlLock, rightControlLock, scheduleLock, pwmLock, printLock, takePicLock, fileWriteLock, writeCountsLock, newPicLock; //FIXME PRINTLOCK
pthread_cond_t willTakePic = PTHREAD_COND_INITIALIZER;

struct publicState
{
    bool end;
    bool stopCollection;
    char rightDirection;
    char leftDirection;
    int left_lvl;
    int right_lvl;
    struct queue leftQueue;
    struct queue rightQueue;
    struct queue inputQueue;
    struct spatial_data sd;
    int readCounts;
    int mode; // 0 = m0, 1 = m1, 2 = m2
    int z_count;
    int c_count;
    bool newPic;   //is there a new pic to process?
    bool printPic; //should we print pic?
    unsigned char image[IMG_RED_WIDTH][IMG_RED_HEIGHT];
} state;

struct dataQueue
{
    struct fQueue ACCEL_X;
    struct fQueue ACCEL_Y;
    struct fQueue ACCEL_Z;
    struct fQueue GYRO_X;
    struct fQueue GYRO_Y;
    struct fQueue GYRO_Z;
    struct fQueue TIME;
} dQueues;
;
;

struct RGB_pixel
{
    unsigned char R;
    unsigned char G;
    unsigned char B;
};

void initDataQueues(struct dataQueue *dq)
{
    initFQueue(&(dq->ACCEL_X), DATA_QUEUE_SIZE);
    initFQueue(&(dq->ACCEL_Y), DATA_QUEUE_SIZE);
    initFQueue(&(dq->ACCEL_Z), DATA_QUEUE_SIZE);
    initFQueue(&(dq->GYRO_X), DATA_QUEUE_SIZE);
    initFQueue(&(dq->GYRO_Y), DATA_QUEUE_SIZE);
    initFQueue(&(dq->GYRO_Z), DATA_QUEUE_SIZE);
    initFQueue(&(dq->TIME), DATA_QUEUE_SIZE);

    float AX[DATA_QUEUE_SIZE];
    float AY[DATA_QUEUE_SIZE];
    float AZ[DATA_QUEUE_SIZE];
    float GX[DATA_QUEUE_SIZE];
    float GY[DATA_QUEUE_SIZE];
    float GZ[DATA_QUEUE_SIZE];
    float RT[DATA_QUEUE_SIZE];

    memset(AX, 0, DATA_QUEUE_SIZE * sizeof(float));
    memset(AY, 0, DATA_QUEUE_SIZE * sizeof(float));
    memset(AZ, 0, DATA_QUEUE_SIZE * sizeof(float));
    memset(GX, 0, DATA_QUEUE_SIZE * sizeof(float));
    memset(GY, 0, DATA_QUEUE_SIZE * sizeof(float));
    memset(GZ, 0, DATA_QUEUE_SIZE * sizeof(float));
    memset(RT, 0, DATA_QUEUE_SIZE * sizeof(float));

    dq->ACCEL_X.contents = AX;
    dq->ACCEL_Y.contents = AY;
    dq->ACCEL_Z.contents = AZ;
    dq->GYRO_X.contents = GX;
    dq->GYRO_Y.contents = GY;
    dq->GYRO_Z.contents = GZ;
    dq->TIME.contents = RT;
}

void clearDataQueues(struct dataQueue *dq)
{
    clearFQueue(&(dq->ACCEL_X));
    clearFQueue(&(dq->ACCEL_Y));
    clearFQueue(&(dq->ACCEL_Z));
    clearFQueue(&(dq->GYRO_X));
    clearFQueue(&(dq->GYRO_Y));
    clearFQueue(&(dq->GYRO_Z));
    clearFQueue(&(dq->TIME));
}
//set all inputs to outputs to turn off lights on quit
void cleanQuit()
{
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT; //GPIO12
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT; //GPIO13
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT; //GPIO05
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT; //GPIO06
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT; //GPIO22
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT; //GPIO23
    raspicam_wrapper_destroy(Camera);
}

void INThandler(int sig)
{
    cleanQuit();
    exit(0);
}

int setSpdLeft(int spd)
{
    int time_taken = 0;

    if (spd < PWM_MIN)
        spd = 0;

    if (spd > state.left_lvl)
    {
        if (spd >= PWM_MAX)
            spd = RIGHT_MAX;

        if (state.left_lvl < PWM_MIN)
            state.left_lvl = PWM_MIN;

        while (state.left_lvl < spd)
        {
            state.left_lvl += PWM_5_PERC;
            io->pwm->DAT1 = leftLookup[state.left_lvl]; //PWMA left
            usleep(RAMP_TIME);
            time_taken += RAMP_TIME;
        }
    }
    else if (spd < state.left_lvl)
    {
        if (state.left_lvl < PWM_MIN)
            state.left_lvl = PWM_MIN;

        while (state.left_lvl > spd)
        {
            state.left_lvl -= PWM_5_PERC;

            if (state.left_lvl < 0)
                state.left_lvl = 0;

            io->pwm->DAT1 = leftLookup[state.left_lvl]; //PWMA left
            usleep(RAMP_TIME);
            time_taken += RAMP_TIME;
        }
    }

    return time_taken;
}

int setSpdRight(int spd)
{
    int time_taken = 0;

    if (spd < PWM_MIN)
        spd = 0;

    if (spd > state.right_lvl)
    {
        if (spd >= RIGHT_MAX)
            spd = RIGHT_MAX;

        if (state.right_lvl < PWM_MIN)
            state.right_lvl = PWM_MIN;

        while (state.right_lvl < spd)
        {
            state.right_lvl += PWM_5_PERC;
            io->pwm->DAT2 = state.right_lvl; //PWMA right
            usleep(RAMP_TIME);
            time_taken += RAMP_TIME;
        }
    }
    else if (spd < state.right_lvl)
    {
        if (state.right_lvl < PWM_MIN)
            state.right_lvl = PWM_MIN;

        while (state.right_lvl > spd)
        {
            state.right_lvl -= PWM_5_PERC;
            if (state.right_lvl < 0)
                state.right_lvl = 0;
            io->pwm->DAT2 = state.right_lvl; //PWMA right
            usleep(RAMP_TIME);
            time_taken += RAMP_TIME;
        }
    }

    return time_taken;
}

void takePic()
{
    raspicam_wrapper_grab(Camera);

    pthread_mutex_lock(&newPicLock);
    state.newPic = true;
    pthread_mutex_unlock(&newPicLock);
}

void dataToPPM(unsigned char *data, char *name, int width, int height, int size) //for testing
{
    FILE *outFile = fopen(name, "wb");
    if (outFile != NULL)
    {
        fprintf(outFile, "P6\n"); // write .ppm file header
        fprintf(outFile, "%d %d 255\n", width, height);
        // write the image data
        fwrite(data, 1, size, outFile);
        fclose(outFile);
        printf("Image, picture saved as %s\n", name);
    }
}

void dataToPGMFormat(unsigned char *data, unsigned char *newData, int width, int height, int pixel_count)
{
    struct RGB_pixel *pixel;
    unsigned int pixel_index;
    unsigned char pixel_value;

    pixel = (struct RGB_pixel *)data; // view data as R-byte, G-byte, and B-byte per pixel

    for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
    {
        newData[pixel_index] = pixel[pixel_index].B; // RGB values are all equal, we can take any
    }
}

void dataToPGM(unsigned char *data, char *name, int width, int height, int pixel_count, int size) //for testing
{
    unsigned char newData[pixel_count];
    dataToPGMFormat(data, newData, width, height, pixel_count);

    FILE *outFile = fopen(name, "wb");
    if (outFile != NULL)
    {
        fprintf(outFile, "P5\n"); // write .pgm file header
        fprintf(outFile, "%d %d 255\n", width, height);
        // write the image data
        fwrite(newData, 1, size / 3, outFile);
        fclose(outFile);
        printf("Image, picture saved as %s\n", name);
    }
}

void makeGrayscale(unsigned char *data, unsigned int pixel_count)
{
    struct RGB_pixel *pixel;
    unsigned int pixel_index;
    unsigned char pixel_value;

    pixel = (struct RGB_pixel *)data; // view data as R-byte, G-byte, and B-byte per pixel

    for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
    {
        // gray scale => average of R color, G color, and B color intensity
        pixel_value = (((unsigned int)(pixel[pixel_index].R)) +
                       ((unsigned int)(pixel[pixel_index].G)) +
                       ((unsigned int)(pixel[pixel_index].B))) /
                      3;                    // do not worry about rounding
        pixel[pixel_index].R = pixel_value; // same intensity for all three color
        pixel[pixel_index].G = pixel_value;
        pixel[pixel_index].B = pixel_value;
    }
}

void processPic(bool printFull, bool printThresh)
{
    size_t image_size = raspicam_wrapper_getImageTypeSize(Camera, RASPICAM_WRAPPER_FORMAT_RGB);
    unsigned char *data = (unsigned char *)malloc(image_size);
    raspicam_wrapper_retrieve(Camera, data, RASPICAM_WRAPPER_FORMAT_RGB);
    unsigned int pixHeight = raspicam_wrapper_getHeight(Camera);
    unsigned int pixWidth = raspicam_wrapper_getWidth(Camera);
    unsigned int pixel_count = pixHeight * pixWidth;
    struct charImg cImg;

    makeGrayscale(data, pixel_count);

    if (printFull)
    {
        dataToPPM(data, "test1.ppm", pixWidth, pixHeight, image_size);              //FIXME
        dataToPGM(data, "test1.pgm", pixWidth, pixHeight, pixel_count, image_size); //FIXME
        unsigned char pgmData[pixel_count];
        dataToPGMFormat(data, pgmData, pixWidth, pixHeight, pixel_count);
        cImg = edge_image(1, pgmData, "edge3.pgm", pixWidth, pixHeight, 1);
    }
    else
    {
        unsigned char pgmData[pixel_count];
        dataToPGMFormat(data, pgmData, pixWidth, pixHeight, pixel_count);
        cImg = edge_image(0, pgmData, "edge.pgm", pixWidth, pixHeight, 1);
    }

    free(data);
}

void *procPic()
{
    long remainingTime = 0, startLoop = 0, stopLoop = 0;
    pthread_mutex_lock(&takePicLock);
    pthread_cond_wait(&willTakePic, &takePicLock);
    pthread_mutex_unlock(&takePicLock);

    ualarm(THREAD_PIC_PROCESS_TIME_uS, THREAD_PIC_PROCESS_TIME_uS); //set alarm

    while (true)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;
        if (state.end)
        {
            ualarm(0, 0); //reset alarm
            break;
        }
        if (state.stopCollection)
        {                 //stop collection if s is hit
            ualarm(0, 0); //reset alarm
            pthread_mutex_lock(&takePicLock);
            pthread_cond_wait(&willTakePic, &takePicLock);
            pthread_mutex_unlock(&takePicLock);

            ualarm(THREAD_PIC_PROCESS_TIME_uS, THREAD_PIC_PROCESS_TIME_uS); //set alarm
        }

        if (state.newPic)
        {
            state.newPic = false;

            pthread_mutex_lock(&printLock);
            bool print = state.printPic;
            processPic(print, print);
            state.printPic = false;
            pthread_mutex_unlock(&printLock);
            // printf("hey");//FIXME
        }

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PIC_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); //sleep for remaining portion of 100ms
    }
    pthread_exit(0);
}

void *scheduler()
{
    long remainingTime, startLoop, stopLoop, startTurn, stopTurn, m0TimerStart, m0TimerStop = 0;
    uint32_t irRightVal;
    uint32_t irLeftVal;
    bool m2IsDriving = false;

    while (1)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;
        if (state.end)
        {
            pthread_cond_broadcast(&willTakePic);
            break;
        }

        char command;

        m0TimerStop = clock() / CLOCKS_PER_MIRCO;
        if (state.mode == 0 && m0TimerStop - m0TimerStart > 5000000)
        {
            state.mode = 1;
            state.stopCollection = true;
        }
        else if ((command = queuePop(&state.inputQueue)) != '\0')
        {
            if (command == 'm')
            {
                while ((command = queuePop(&state.inputQueue)) == '\0')
                    ;

                if (command == '0')
                {
                    // calibrate_accelerometer_and_gyroscope(&calibration_accelerometer, &calibration_gyroscope, io->bsc);
                    m2IsDriving = false;
                    state.mode = 0;
                    state.stopCollection = false;

                    pthread_mutex_lock(&rightControlLock);
                    clearQueue(&state.rightQueue);
                    queuePush('s', &state.rightQueue);
                    pthread_mutex_unlock(&rightControlLock);
                    pthread_mutex_lock(&leftControlLock);
                    clearQueue(&state.leftQueue);
                    queuePush('s', &state.leftQueue);
                    pthread_mutex_unlock(&leftControlLock);

                    pthread_cond_broadcast(&willTakePic);
                    m0TimerStart = clock() / CLOCKS_PER_MIRCO;
                }
                else if (command == '1')
                {
                    // calibrate_accelerometer_and_gyroscope(&calibration_accelerometer, &calibration_gyroscope, io->bsc);
                    m2IsDriving = false;
                    state.stopCollection = false;
                    state.mode = 1;
                    pthread_mutex_lock(&rightControlLock);
                    clearQueue(&state.rightQueue);
                    queuePush('s', &state.rightQueue);
                    pthread_mutex_unlock(&rightControlLock);
                    pthread_mutex_lock(&leftControlLock);
                    clearQueue(&state.leftQueue);
                    queuePush('s', &state.leftQueue);
                    pthread_mutex_unlock(&leftControlLock);
                }
                else if (command == '2')
                {
                    // calibrate_accelerometer_and_gyroscope(&calibration_accelerometer, &calibration_gyroscope, io->bsc);
                    m2IsDriving = false;
                    state.stopCollection = false;
                    state.mode = 2;

                    pthread_mutex_lock(&rightControlLock);
                    clearQueue(&state.rightQueue);
                    queuePush('s', &state.rightQueue);
                    pthread_mutex_unlock(&rightControlLock);
                    pthread_mutex_lock(&leftControlLock);
                    clearQueue(&state.leftQueue);
                    queuePush('s', &state.leftQueue);
                    pthread_mutex_unlock(&leftControlLock);

                    GPIO_SET(io->gpio, 5); //set to forward
                    GPIO_CLR(io->gpio, 6);
                    GPIO_SET(io->gpio, 22); //set to forward
                    GPIO_CLR(io->gpio, 23);
                }
                else
                {
                    printf("\n\033[0;33mhw8>\033[0m ");
                    printf("m%c is not a command", command);
                }
            }

            else if (state.mode == 1)
            {
                if (command == 's')
                {
                    state.stopCollection = true;
                    state.mode = 1;
                    pthread_mutex_lock(&rightControlLock);
                    clearQueue(&state.rightQueue);
                    queuePush('s', &state.rightQueue);
                    pthread_mutex_unlock(&rightControlLock);
                    pthread_mutex_lock(&leftControlLock);
                    clearQueue(&state.leftQueue);
                    queuePush('s', &state.leftQueue);
                    pthread_mutex_unlock(&leftControlLock);
                }
                else if (command == 'w' && state.leftDirection != 'w' && state.rightDirection != 'w')
                {
                    state.stopCollection = false;
                    // pthread_cond_broadcast(&willTakePic);
                }

                pthread_mutex_lock(&leftControlLock);
                queuePush(command, &state.leftQueue);
                pthread_mutex_unlock(&leftControlLock);

                pthread_mutex_lock(&rightControlLock);
                queuePush(command, &state.rightQueue);
                pthread_mutex_unlock(&rightControlLock);
            }
        }

        if (state.mode == 2)
        {
            if (command == 's')
            {
                m2IsDriving = false;
                state.stopCollection = true;
                state.mode = 2;
                pthread_mutex_lock(&rightControlLock);
                clearQueue(&state.rightQueue);
                queuePush('s', &state.rightQueue);
                pthread_mutex_unlock(&rightControlLock);
                pthread_mutex_lock(&leftControlLock);
                clearQueue(&state.leftQueue);
                queuePush('s', &state.leftQueue);
                pthread_mutex_unlock(&leftControlLock);
            }
            else if (command == 'w')
            { //FIXME
                state.stopCollection = false;
                pthread_cond_broadcast(&willTakePic);
            }
            else if (command == 'p')
            {
                pthread_mutex_lock(&printLock);
                state.printPic = true;
                pthread_mutex_unlock(&printLock);
            }
            // else if (command == 'w' && state.leftDirection != 'w' && state.rightDirection != 'w')
            // {
            //     // calibrate_accelerometer_and_gyroscope(&calibration_accelerometer, &calibration_gyroscope, io->bsc);
            //     m2IsDriving = true;
            //     pthread_cond_broadcast(&willTakePic);

            //     GPIO_SET(io->gpio, 5); //set to forward
            //     GPIO_CLR(io->gpio, 6);
            //     GPIO_SET(io->gpio, 22); //set to forward
            //     GPIO_CLR(io->gpio, 23);
            //     state.leftDirection = 'w';
            //     state.rightDirection = 'w';

            //     setSpdLeft(PWM_MIN + PWM_5_PERC);
            //     setSpdRight(PWM_MIN + PWM_5_PERC);
            // }

            // if (m2IsDriving)
            // {

            //     irRightVal = GPIO_READ(io->gpio, 25);
            //     irLeftVal = GPIO_READ(io->gpio, 24);
            //     // printf("right %d\n", irRightVal);
            //     // printf("left %d\n", irLeftVal);

            //     stopTurn = clock() / CLOCKS_PER_MIRCO;

            //     if (stopTurn - startTurn > CHECK_IR_INT)
            //     {
            //         if (irRightVal != 0 && state.c_count < 5)
            //         {
            //             if (state.z_count > 0) //is currently turning right, reset and turn left
            //             {
            //                 pthread_mutex_lock(&rightControlLock);
            //                 pthread_mutex_lock(&leftControlLock);
            //                 clearQueue(&state.rightQueue);
            //                 clearQueue(&state.leftQueue);
            //                 pthread_mutex_unlock(&rightControlLock);
            //                 pthread_mutex_unlock(&leftControlLock);
            //                 state.z_count = 0;
            //             }
            //             pthread_mutex_lock(&rightControlLock);
            //             queuePush('c', &state.rightQueue);
            //             pthread_mutex_unlock(&rightControlLock);
            //             pthread_mutex_lock(&leftControlLock);
            //             queuePush('c', &state.leftQueue);
            //             pthread_mutex_unlock(&leftControlLock);
            //             startTurn = clock() / CLOCKS_PER_MIRCO;
            //             // printf("right %d\n", irRightVal);
            //             // printf("left %d\n", irLeftVal);
            //             state.c_count += 1;
            //         }
            //         if (irLeftVal != 0 && state.z_count < 5)
            //         {
            //             if (state.c_count > 0) //is currently turning right, reset and turn left
            //             {
            //                 pthread_mutex_lock(&rightControlLock);
            //                 pthread_mutex_lock(&leftControlLock);
            //                 clearQueue(&state.rightQueue);
            //                 clearQueue(&state.leftQueue);
            //                 pthread_mutex_unlock(&rightControlLock);
            //                 pthread_mutex_unlock(&leftControlLock);
            //                 state.c_count = 0;
            //             }
            //             pthread_mutex_lock(&rightControlLock);
            //             queuePush('z', &state.rightQueue);
            //             pthread_mutex_unlock(&rightControlLock);
            //             pthread_mutex_lock(&leftControlLock);
            //             queuePush('z', &state.leftQueue);
            //             pthread_mutex_unlock(&leftControlLock);
            //             startTurn = clock() / CLOCKS_PER_MIRCO;
            //             // printf("right %d\n", irRightVal);
            //             // printf("left %d\n", irLeftVal);
            //             state.z_count += 1;
            //         }
            //     }
            // }
        }

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); //sleep for remaining portion of 10ms
    }
    pthread_exit(0);
}

//controls left motors
void *leftCtrl()
{
    char nextCommand;
    long remainingTime, startLoop, stopLoop;
    while (1)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;

        if (state.end)
        {
            break;
        }

        pthread_mutex_lock(&leftControlLock);
        nextCommand = queuePop(&state.leftQueue);
        pthread_mutex_unlock(&leftControlLock);

        if (nextCommand == 'i')
        {
            if (state.left_lvl >= RIGHT_MAX)
                state.left_lvl = RIGHT_MAX;
            else if (state.left_lvl == 0)
                state.left_lvl = PWM_MIN;
            else
                state.left_lvl += PWM_5_PERC;

            pthread_mutex_lock(&pwmLock);
            io->pwm->DAT1 = leftLookup[state.left_lvl]; //PWMB left
            pthread_mutex_unlock(&pwmLock);
        }
        else if (nextCommand == 'j')
        {
            if (state.left_lvl <= PWM_MIN)
                state.left_lvl = 0;
            else
                state.left_lvl -= PWM_5_PERC;

            pthread_mutex_lock(&pwmLock);
            io->pwm->DAT1 = leftLookup[state.left_lvl]; //PWMB left
            pthread_mutex_unlock(&pwmLock);
        }
        else if (nextCommand == 'w')
        {
            int tmpLvl = state.left_lvl;
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 5); //set to forward
                GPIO_CLR(io->gpio, 6);

                state.left_lvl = tmpLvl / 2;
                io->pwm->DAT1 = state.left_lvl; //PWMB left
                usleep(100000);                 // sleep .1 s
                state.left_lvl = tmpLvl;
                io->pwm->DAT1 = state.left_lvl; //PWMB left
            }
            if (state.leftDirection == 'x')
            {
                state.left_lvl = state.left_lvl / 2;
                io->pwm->DAT1 = state.left_lvl;
                ;               //PWMB left
                usleep(100000); // sleep .1 s

                state.left_lvl = 0;             //stop
                io->pwm->DAT1 = state.left_lvl; //PWMB left
                usleep(100000);                 //sleep .1s

                GPIO_SET(io->gpio, 5); //set to forward
                GPIO_CLR(io->gpio, 6);

                state.left_lvl = tmpLvl / 2;    // half speed
                io->pwm->DAT1 = state.left_lvl; //PWMB left
                usleep(100000);                 //sleep .1s
                state.left_lvl = tmpLvl;        // half speed
                io->pwm->DAT1 = state.left_lvl;
                ; //PWMB left
            }
            else
            {
                GPIO_SET(io->gpio, 5); //set to forward
                GPIO_CLR(io->gpio, 6);
            }
            state.leftDirection = 'w';
        }
        else if (nextCommand == 'x')
        {
            int tmpLvl = state.left_lvl;
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 6); //set to reverse
                GPIO_CLR(io->gpio, 5);

                state.left_lvl = state.left_lvl / 2;
                io->pwm->DAT1 = state.left_lvl;
                ;               //PWMB left
                usleep(100000); // sleep .1 s
                state.left_lvl = tmpLvl;
                io->pwm->DAT1 = state.left_lvl;
                ; //PWMB left
            }
            if (state.leftDirection == 'w')
            {
                state.left_lvl = state.left_lvl / 2;
                io->pwm->DAT1 = state.left_lvl; //PWMB left
                usleep(100000);                 // sleep .1 s

                state.left_lvl = 0;             //stop
                io->pwm->DAT1 = state.left_lvl; //PWMB left
                usleep(100000);                 //sleep .1s

                GPIO_SET(io->gpio, 6); //set to reverse
                GPIO_CLR(io->gpio, 5);

                state.left_lvl = tmpLvl / 2;    // half speed
                io->pwm->DAT1 = state.left_lvl; //PWMB left
                usleep(100000);                 //sleep .1s
                state.left_lvl = tmpLvl;        // half speed
                io->pwm->DAT1 = state.left_lvl; //PWMB left
            }
            else
            {
                GPIO_SET(io->gpio, 6); //set to reverse
                GPIO_CLR(io->gpio, 5);
            }
            state.leftDirection = 'x';
        }
        else if (nextCommand == 's')
        {
            state.left_lvl = state.left_lvl / 2;
            io->pwm->DAT1 = state.left_lvl; //PWMB left
            usleep(100000);                 // sleep .1 s
            state.left_lvl = 0;             // stop
            io->pwm->DAT1 = state.left_lvl;
            ; //PWMB left

            GPIO_CLR(io->gpio, 6); //set to stop
            GPIO_CLR(io->gpio, 5);
            state.leftDirection = 's';
        }
        else if (nextCommand == 'z')
        {
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 5); //set to forward
                GPIO_CLR(io->gpio, 6);
            }
            int tmpLvl = state.left_lvl;

            setSpdLeft(0);
            usleep(Z_C_TURN_TIME);
            setSpdLeft(tmpLvl);

            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 5); //set to stop
                GPIO_CLR(io->gpio, 6);
            }
            if (state.z_count > 0)
                state.z_count -= 1;
        }
        else if (nextCommand == 'c')
        {
            int turn_time = Z_C_TURN_TIME;

            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 5); //set to forward
                GPIO_CLR(io->gpio, 6);
            }
            int tmpLvl = state.left_lvl;

            setSpdLeft(PWM_MAX);

            usleep(turn_time);

            setSpdLeft(tmpLvl);
            if (state.leftDirection == 's')
            {
                GPIO_CLR(io->gpio, 5); //set to stop
                GPIO_CLR(io->gpio, 6);
            }
            if (state.c_count > 0)
                state.c_count -= 1;
        }
        else if (nextCommand == 'a')
        {
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 5); //set to forward
                GPIO_CLR(io->gpio, 6);
            }
            int tmpLvl = state.left_lvl;
            state.left_lvl = leftLookup[PWM_MIN];
            io->pwm->DAT1 = state.left_lvl; //PWMB right
            usleep(TURN_TIME);
            state.left_lvl = tmpLvl;
            io->pwm->DAT1 = state.left_lvl; //PWMB right
            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 5); //set to stop
                GPIO_CLR(io->gpio, 6);
            }
        }
        else if (nextCommand == 'd')
        {
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 5); //set to forward
                GPIO_CLR(io->gpio, 6);
            }
            int tmpLvl = state.left_lvl;
            state.left_lvl = PWM_MAX;       //adfsaf
            io->pwm->DAT1 = state.left_lvl; //PWMB right
            usleep(TURN_TIME);
            state.left_lvl = tmpLvl;
            io->pwm->DAT1 = state.left_lvl; //PWMB right
            if (state.leftDirection == 's')
            {
                GPIO_CLR(io->gpio, 5); //set to stop
                GPIO_CLR(io->gpio, 6);
            }
        }

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); //sleep for remaining portion of 10ms
    }
    pthread_exit(0);
}

void *rightCtrl()
{
    char nextCommand;
    long remainingTime, startLoop, stopLoop;
    while (1)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;

        if (state.end)
        {
            break;
        }

        pthread_mutex_lock(&rightControlLock);
        nextCommand = queuePop(&state.rightQueue);
        pthread_mutex_unlock(&rightControlLock);

        if (nextCommand == 'i')
        {
            if (state.right_lvl >= RIGHT_MAX)
                state.right_lvl = RIGHT_MAX;
            else if (state.right_lvl == 0)
                state.right_lvl = PWM_MIN;
            else
                state.right_lvl += PWM_5_PERC;

            pthread_mutex_lock(&pwmLock);
            io->pwm->DAT2 = state.right_lvl; //PWMB right
            pthread_mutex_unlock(&pwmLock);
        }
        else if (nextCommand == 'j')
        {
            if (state.right_lvl <= PWM_MIN)
                state.right_lvl = 0;
            else
                state.right_lvl -= PWM_5_PERC;

            pthread_mutex_lock(&pwmLock);
            io->pwm->DAT2 = state.right_lvl; //PWMB right
            pthread_mutex_unlock(&pwmLock);
        }
        else if (nextCommand == 'w')
        {
            int tmpLvl = state.right_lvl;
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 22); //set to forward
                GPIO_CLR(io->gpio, 23);

                state.right_lvl = tmpLvl / 2;
                io->pwm->DAT1 = state.right_lvl; //PWMB right
                usleep(100000);                  // sleep .1 s
                state.right_lvl = tmpLvl;
                io->pwm->DAT1 = state.right_lvl; //PWMB right
            }
            if (state.rightDirection == 'x')
            {
                state.right_lvl = state.right_lvl / 2;
                io->pwm->DAT1 = state.right_lvl;
                ;               //PWMB right
                usleep(100000); // sleep .1 s

                state.right_lvl = 0;             //stop
                io->pwm->DAT1 = state.right_lvl; //PWMB right
                usleep(100000);                  //sleep .1s

                GPIO_SET(io->gpio, 22); //set to forward
                GPIO_CLR(io->gpio, 23);

                state.right_lvl = tmpLvl / 2;    // half speed
                io->pwm->DAT1 = state.right_lvl; //PWMB right
                usleep(100000);                  //sleep .1s
                state.right_lvl = tmpLvl;        // half speed
                io->pwm->DAT1 = state.right_lvl;
                ; //PWMB right
            }
            else
            {
                GPIO_SET(io->gpio, 22); //set to forward
                GPIO_CLR(io->gpio, 23);
            }
            state.rightDirection = 'w';
        }
        else if (nextCommand == 'x')
        {
            int tmpLvl = state.right_lvl;
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 23); //set to reverse
                GPIO_CLR(io->gpio, 22);

                state.right_lvl = state.right_lvl / 2;
                io->pwm->DAT1 = state.right_lvl;
                ;               //PWMB right
                usleep(100000); // sleep .1 s
                state.right_lvl = tmpLvl;
                io->pwm->DAT1 = state.right_lvl;
                ; //PWMB right
            }
            if (state.rightDirection == 'w')
            {
                state.right_lvl = state.right_lvl / 2;
                io->pwm->DAT1 = state.right_lvl; //PWMB right
                usleep(100000);                  // sleep .1 s

                state.right_lvl = 0;             //stop
                io->pwm->DAT1 = state.right_lvl; //PWMB right
                usleep(100000);                  //sleep .1s

                GPIO_SET(io->gpio, 23); //set to reverse
                GPIO_CLR(io->gpio, 22);

                state.right_lvl = tmpLvl / 2;    // half speed
                io->pwm->DAT1 = state.right_lvl; //PWMB right
                usleep(100000);                  //sleep .1s
                state.right_lvl = tmpLvl;        // half speed
                io->pwm->DAT1 = state.right_lvl; //PWMB right
            }
            else
            {
                GPIO_SET(io->gpio, 23); //set to reverse
                GPIO_CLR(io->gpio, 22);
            }
            state.rightDirection = 'x';
        }
        else if (nextCommand == 's')
        {
            state.right_lvl = state.right_lvl / 2;
            io->pwm->DAT2 = state.right_lvl; //PWMB right
            usleep(100000);                  // sleep .1 s
            state.right_lvl = 0;             // stop
            io->pwm->DAT2 = state.right_lvl; //PWMB right

            GPIO_CLR(io->gpio, 23); //set to stop
            GPIO_CLR(io->gpio, 22);
            state.rightDirection = 's';
        }
        else if (nextCommand == 'c')
        {
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 22); //set to forward
                GPIO_CLR(io->gpio, 23);
            }
            int tmpLvl = state.right_lvl;

            setSpdRight(0);

            usleep(Z_C_TURN_TIME);

            setSpdRight(tmpLvl);

            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 22); //set to stop
                GPIO_CLR(io->gpio, 23);
            }
            if (state.c_count > 0)
                state.c_count -= 1;
        }
        else if (nextCommand == 'z')
        {
            int turn_time = Z_C_TURN_TIME;
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 22); //set to forward
                GPIO_CLR(io->gpio, 23);
            }
            int tmpLvl = state.right_lvl;

            setSpdRight(PWM_MAX);

            usleep(turn_time);

            setSpdRight(tmpLvl);

            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 22); //set to stop
                GPIO_CLR(io->gpio, 23);
            }
            if (state.z_count > 0)
                state.z_count -= 1;
        }
        else if (nextCommand == 'd')
        {
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 22); //set to forward
                GPIO_CLR(io->gpio, 23);
            }
            int tmpLvl = state.right_lvl;
            state.right_lvl = PWM_MIN + 3 * PWM_5_PERC;
            io->pwm->DAT2 = state.right_lvl; //PWMB right
            usleep(TURN_TIME);
            state.right_lvl = tmpLvl;
            io->pwm->DAT2 = state.right_lvl; //PWMB right
            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 22); //set to stop
                GPIO_CLR(io->gpio, 23);
            }
        }
        else if (nextCommand == 'a')
        {
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 22); //set to forward
                GPIO_CLR(io->gpio, 23);
            }
            int tmpLvl = state.right_lvl;
            state.right_lvl = RIGHT_MAX;
            io->pwm->DAT2 = state.right_lvl; //PWMB right
            usleep(TURN_TIME);
            state.right_lvl = tmpLvl;
            io->pwm->DAT2 = state.right_lvl; //PWMB right
            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 22); //set to stop
                GPIO_CLR(io->gpio, 23);
            }
        }

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); //sleep for remaining portion of 10ms
    }

    pthread_exit(0);
}

//thread fuction to check for input
void *checkInput()
{
    char tempInput;
    while (1)
    {
        while ((tempInput = (char)getchar()) == '\0') //while input is not empty do nothing
            ;

        if (tempInput == 't' || tempInput == 'n' || tempInput == 'p' || tempInput == 's' || tempInput == 'w' || tempInput == 'x' || tempInput == 'i' || tempInput == 'j' || tempInput == 'a' || tempInput == 'd' || tempInput == 'q' || tempInput == 'm' || tempInput == '1' || tempInput == '2' || tempInput == '0' || tempInput == 'z' || tempInput == 'c')
        {
            if (tempInput == 'q')
            {
                state.end = true;
                break;
            }
            if (tempInput == 'm')
            {
                pthread_mutex_lock(&scheduleLock);
                queuePush(tempInput, &state.inputQueue);
                pthread_mutex_unlock(&scheduleLock);

                while ((tempInput = (char)getchar()) == '\0') //while input is not empty do nothing
                    ;
            }

            pthread_mutex_lock(&scheduleLock);
            queuePush(tempInput, &state.inputQueue);
            pthread_mutex_unlock(&scheduleLock);
        }

        printf("\n\033[0;33mhw8>\033[0m "); //newline for reading
    }

    pthread_exit(0);
}

int main(void)
{
    signal(SIGINT, INThandler);
    signal(SIGALRM, takePic);

    io = import_registers();
    Camera = raspicam_wrapper_create();

    if (raspicam_wrapper_open(Camera))
    {
        printf("opening camera");
        sleep(1);
    }
    else
        printf("error opening camera\n");

    if (io != NULL)
    {
        /* print where the I/O memory was actually mapped to */
        printf("mem at 0x%8.8X\n", (unsigned int)io);

        /* set the pin function to OUTPUT for GPIO */
        enable_pwm_clock(io->cm, io->pwm);
        pwm_init(io, PWM_MAX);

        io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
        io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;

        io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT; // AI1
        io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT; // AI2
        io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT; // BI1
        io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT; // BI2

        GPIO_CLR(io->gpio, 5); //init to stop
        GPIO_CLR(io->gpio, 6);
        GPIO_CLR(io->gpio, 22);
        GPIO_CLR(io->gpio, 23);

        printf("hit 'ctl c' or 'q' to quit\n");
        printf("\033[0;33mhw8>\033[0m "); //newline for reading

        static struct termios attr;
        tcgetattr(STDIN_FILENO, &attr);
        attr.c_lflag &= ~(ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &attr);

        struct publicState cArgs;

        state.end = false;
        state.stopCollection = true;
        state.left_lvl = 0;
        state.right_lvl = 0;
        state.rightDirection = 's';
        state.leftDirection = 's';
        state.z_count = 0;
        state.c_count = 0;

        leftLookup[260] = 297;
        leftLookup[297] = 365;
        leftLookup[334] = 439;
        leftLookup[371] = 455;
        leftLookup[408] = 506;
        leftLookup[445] = 564;
        leftLookup[482] = 601;
        leftLookup[519] = 624;
        leftLookup[556] = 675;
        leftLookup[593] = 712;
        leftLookup[630] = 753;
        leftLookup[667] = 800;
        leftLookup[704] = 884;
        leftLookup[741] = 867;
        leftLookup[778] = 890;
        leftLookup[815] = 913;
        leftLookup[852] = 957;
        leftLookup[889] = 980;
        leftLookup[926] = 1000;

        char leftContents[INSTR_QUEUE_SIZE];
        memset(leftContents, 0, INSTR_QUEUE_SIZE * sizeof(char));
        char rightContents[INSTR_QUEUE_SIZE];
        memset(rightContents, 0, INSTR_QUEUE_SIZE * sizeof(char));
        char inputContents[INSTR_QUEUE_SIZE];
        memset(inputContents, 0, INSTR_QUEUE_SIZE * sizeof(char));

        state.leftQueue.contents = leftContents;
        state.rightQueue.contents = rightContents;
        state.inputQueue.contents = inputContents;
        state.mode = 1; //FIXME true

        initDataQueues(&dQueues);

        initQueue(&state.leftQueue, INSTR_QUEUE_SIZE);
        initQueue(&state.rightQueue, INSTR_QUEUE_SIZE);
        initQueue(&state.inputQueue, INSTR_QUEUE_SIZE);

        pthread_attr_t pAttr;
        pthread_attr_init(&pAttr);

        pthread_mutex_init(&scheduleLock, NULL);
        pthread_mutex_init(&inputLock, NULL);
        pthread_mutex_init(&leftControlLock, NULL);
        pthread_mutex_init(&rightControlLock, NULL);
        pthread_mutex_init(&pwmLock, NULL);
        pthread_mutex_init(&printLock, NULL);
        pthread_mutex_init(&takePicLock, NULL);
        pthread_mutex_init(&fileWriteLock, NULL);
        pthread_mutex_init(&writeCountsLock, NULL);
        pthread_mutex_init(&newPicLock, NULL);

        pthread_t inputThread, scheduleThread, leftThread, rightThread, dataCThread, dataAThread, procPicThread;
        pthread_create(&inputThread, &pAttr, &checkInput, NULL);   //create thread for input polling
        pthread_create(&scheduleThread, &pAttr, &scheduler, NULL); //create thread for menu handling
        pthread_create(&leftThread, &pAttr, &leftCtrl, NULL);      //create thread for orange blue pwm control
        pthread_create(&rightThread, &pAttr, &rightCtrl, NULL);    //create thread for orange blue pwm control
        pthread_create(&procPicThread, &pAttr, &procPic, NULL);    //create thread for processing img
        // pthread_create(&dataCThread, &pAttr, &dataCollect, NULL);  //create thread data collection
        // pthread_create(&dataAThread, &pAttr, &dataAnalyze, NULL);  //create thread data collection

        pthread_join(inputThread, NULL);
        pthread_join(scheduleThread, NULL);
        pthread_join(leftThread, NULL);
        pthread_join(rightThread, NULL);
        pthread_join(procPicThread, NULL);
        // pthread_join(dataCThread, NULL);
        // pthread_join(dataAThread, NULL);

        pthread_mutex_destroy(&inputLock);
        pthread_mutex_destroy(&scheduleLock);
        pthread_mutex_destroy(&leftControlLock);
        pthread_mutex_destroy(&rightControlLock);
        pthread_mutex_destroy(&printLock);
        pthread_mutex_destroy(&pwmLock);
        pthread_mutex_destroy(&takePicLock);
        pthread_mutex_destroy(&fileWriteLock);
        pthread_mutex_destroy(&writeCountsLock);
        pthread_mutex_destroy(&newPicLock);

        //reference

        takePic();              //FIXME
        processPic(true, true); //FIXME

        cleanQuit(); //turn off all lights and quit
    }
}
