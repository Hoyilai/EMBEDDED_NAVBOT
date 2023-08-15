/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 8 
* Revision V1.1
* On 4/28/2022
* By Hongyi Lai
* 
***************************************************/
 
/* Homework 8 Car Self driving program
 * m1 to manually drive the car 
 * m2 for auto-pliot around line using camera
 * 
 */


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/time.h>
#include <math.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "uart.h"
#include "raspicam_wrapper.h"
#include "pixel_format_RGB.h"
#include "scale_image_data.h"


#define APB_CLOCK 250000000
#define CS_PIN  8

#define ROUND_DIVISION(x,y) (((x) + (y)/2)/(y))


struct RGB_pixel
            {
                unsigned char R;
                unsigned char G;
                unsigned char B;
            };

struct pause_flag
{
  pthread_mutex_t lock;
  bool            pause;
};

struct done_flag
{
  pthread_mutex_t lock;
  bool            done;
};

struct thread_parameter
{
  volatile struct gpio_register * gpio;
  volatile struct pwm_register  * pwm;
  int                             pin1;
  int                             pin2;
  struct pause_flag *             pause;
  struct done_flag *              done;
};

struct key_thread_parameter
{
  struct done_flag *  done;
  struct pause_flag * pause1;
  struct pause_flag * pause2;
  struct pause_flag * pause3;
  struct pause_flag * pause4;
};
  

//global variables to be used
int forward1 = -1;
int backward1 = -1;
int stop1 = -1;
bool done = false;
bool m1 = false;
bool m2 = false;
bool m0 = false;
#define PWM_RANGE 100
bool rightSensor = false;
bool leftSensor = false;

int rightSwitchVar = -1;
int leftSwitchVar = -1;
//keep track of duty level for each side
int dutyLevelLeft = 0;
int dutyLevelRight = 0;
int dutyLevelLeftHold = 0;
int dutyLevelRightHold = 0;

int  Tstep = 50;  /* PWM time resolution, number used for usleep(Tstep) */
int  Tlevel = 1;  /* repetition count of each light level, eg. repeat 12% light level for 5 times. */
//keep track of how many times left/right were hit
int leftCount;
int rightCount;

//for camera
unsigned char *data;
struct raspicam_wrapper_handle *  Camera;
struct pixel_format_RGB *         scaled_RGB_data;
unsigned char *                   scaled_data;
unsigned int                      scaled_height;
unsigned int                      scaled_width;
int newPicture = 0;
int straight = 1;
int left = -1;
int right = -1;
  



//left motor control function
void *leftHandle( void * arg )
{
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  int pin1 = parameter->pin1;
  int pin2 = parameter->pin2;
  volatile struct io_peripherals *io;
  int currentD;
  io = import_registers();
    //printf("got here forward");
  //enable_pwm_clock( io );
  //printf("left handle thread created");
do
  {
    //printf("left handle thread");
    switch (leftSwitchVar)
    {
      //stop 
      case 0:
    //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((backward1 == 1) || (forward1 ==1)) {
        backward1 = 0;
        forward1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      //set pins to stop (00)
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);
      leftSwitchVar = -1;
      break;
      //forward
      case 1: 
        //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((backward1 == 1) || (stop1 ==1))  {
        backward1 = 0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      //set pins to forward (10)
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);  
      leftSwitchVar = -1;
      break;

      //backward
      case 2:
      //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((forward1 == 1) || (stop1 ==1)) {
        forward1 =0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      //set pins to backwards (01) 
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_SET(&(io->gpio), pin2);
      leftSwitchVar = -1;
      break;
      //faster
      case 3:
      //get current duty level
      currentD = dutyLevelLeft;
      //add 15 (one step)
      currentD = currentD + 15;
      if (currentD >=100){
        currentD = 100;
      }
      //set PWM to this level
      io->pwm.DAT2 = currentD;
      //save updated level to global
      dutyLevelLeft = currentD;
      leftSwitchVar = -1;
      break;
      //slower
      case 4:
      //get current duty level
      currentD = dutyLevelLeft;
      //subtract 15 (one step)
      currentD = currentD - 15;
      if (currentD <=0){
        currentD = 0;
      }
      //set PWM to this level
      io->pwm.DAT2 = currentD;
      //save updated level to global
      dutyLevelLeft = currentD;
      leftSwitchVar = -1;
      break;
      //left
      case 5:
      //decrease left motor 
       dutyLevelLeftHold = dutyLevelLeft;
      while ((leftCount > 0) && (!done)){
        //decrease to 0
        io->pwm.DAT2 = 40;
        io->pwm.DAT1 = 100;
        // sleep for .5 secs
        usleep(500000);
        leftCount = leftCount-1;
      }
      //go back to previous duty level 
      io->pwm.DAT2 = dutyLevelLeftHold;
      leftSwitchVar = -1;
      break;
      //right
      case 6:
      leftSwitchVar = -1;
      break;

      case 7:
      if ((backward1 == 1) || (stop1 ==1))  {
        backward1 = 0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      forward1 = 1;
      //set pins to forward (10)
      GPIO_SET(&(io->gpio), 5);
      GPIO_CLR(&(io->gpio), 6);  
      GPIO_SET(&(io->gpio), 23);
      GPIO_CLR(&(io->gpio), 22); 
      //set duty level to 40
      io->pwm.DAT2= 40;
      io->pwm.DAT1= 40;
      //check camera for black line  
      while ((!done) && (leftSwitchVar ==7)) {
        if ((straight == 1) && (!done) && (leftSwitchVar ==7) ){
          //printf("\n straight\n");
           io->pwm.DAT2= 40;
           io->pwm.DAT1= 40;
            //usleep(1000);
        }
        else if ((left ==1) && (!done) && (leftSwitchVar ==7) ){
          //turn left
          //decrease to 0
          //printf("\n left\n");
          io->pwm.DAT2 = 0;
          io->pwm.DAT1 = 60;
          // sleep for .1 secs
          //usleep(100000);
        //go back to previous duty level 
        }
        else if ((right == 1) && (!done)&& (rightSwitchVar ==7)) {
          //turn right
          //decrease to 0
          //printf("\n right\n");
          io->pwm.DAT2 = 60;
          io->pwm.DAT1 = 0;
        //go back to previous duty level 
        }
      //io->pwm.DAT2= 50;
      //io->pwm.DAT1 = 50;
      }

      case -1:
      break;

      default:
        break;
    }
  } while (!done);

  printf( "Left thread exiting\n" );
  return (void *)0;
}

//right motor control function
void *rightHandle( void * arg )
{
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  int pin1 = parameter->pin1;
  int pin2 = parameter->pin2;
  volatile struct io_peripherals *io;
  int currentD;
  io = import_registers();
  //enable_pwm_clock( io );

  //printf("right handle thread created");
do
  {
    //printf("%i",switchVar);
    switch (rightSwitchVar)
    {
      
      //stop 
      case 0:
        //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((backward1 == 1) || (forward1 ==1)) {
        backward1 = 0;
        forward1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      //set pins to stop (00)
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);
      rightSwitchVar = -1;
      break;

      //forward
      case 1: 
        //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((backward1 == 1) || (stop1 ==1))  {
        backward1 = 0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      //set pins to forward (10)
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2); 
      //printf("car set to forward"); 
      rightSwitchVar = -1;
      break;
 
      //backward
      case 2:
      //(AI1, AI2: 10=>forward, 01=>backward, 00=>stop, 11=>short break
      if ((forward1 == 1) || (stop1 ==1)) {
        forward1 =0;
        stop1 = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      //set pins to backwards (01) 
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_SET(&(io->gpio), pin2);
      rightSwitchVar = -1;
      break;
      //faster
      case 3:
      //get current duty level
      currentD = dutyLevelRight;
      //add 15 (one step)
      currentD = currentD + 15;
      if (currentD >=100){
        currentD = 100;
      }
      //set PWM to this level
      io->pwm.DAT1 = currentD;
      //save updated level to global
      dutyLevelRight = currentD;
      rightSwitchVar = -1;
      break;
      //slower
      case 4:
      //get current duty level
      currentD = dutyLevelRight;
      //add 15 (one step)
      currentD = currentD - 15;
      if (currentD <=0){
        currentD = 0;
      }
      //set PWM to this level
      io->pwm.DAT1 = currentD;
      //save updated level to global
      dutyLevelRight = currentD;
      rightSwitchVar = -1;
      break;
      //left
      case 5:
      rightSwitchVar = -1;
      break;
      //right
      case 6:
      //decrease right motor 
      dutyLevelRightHold = dutyLevelRight;
      while ((rightCount > 0) && (!done)){
        //decrease to 40
        io->pwm.DAT1 = 40;
        io->pwm.DAT2 = 100;
        // sleep for .5 secs
        usleep(500000);
        rightCount = rightCount-1;
      }
      //go back to previous duty level 
      io->pwm.DAT1= dutyLevelRightHold;
      rightSwitchVar = -1;
      break;
      //line tracing mode
      case 7:
      break;
      case -1:
      break;

      default:
        break;
    }
  } while (!done);

  printf( "Right thread exiting\n" );
  return (void *)0;
}
//image processing function
void * imageProcessHandle( void * arg )
{
  int black1 = 0;

      //wait for camera to init
      sleep(4);
      while (!done) {
        
        if (newPicture ==1) {
        // convert to gray-scale
        struct RGB_pixel* pixel;
        //unsigned int      pixel_count;
        //unsigned int      pixel_index;
        unsigned char     pixel_value;
        pixel = (struct RGB_pixel *)scaled_data;
        //printf("\nheight %i", scaled_height); //240
        //printf("\nwidth %i", scaled_width); //320
        //to hold one row of pixels for analysis
        unsigned int      oneRow[scaled_width-1];
        //pixel_count = scaled_height * scaled_width;
        //for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
         // {
          //  pixelCounter = pixelCounter +1;
            //printf("in for loop");
          //  pixel_value = (((unsigned int)(pixel[pixel_index].R)) +
        //                  ((unsigned int)(pixel[pixel_index].G)) +
           //               ((unsigned int)(pixel[pixel_index].B))) / 3; // do not worry about rounding
           // pixel[pixel_index].R = pixel_value;
           // pixel[pixel_index].G = pixel_value;
           // pixel[pixel_index].B = pixel_value;
            //printf("\n index:  %i \n", pixel_index);
            
            //printf("red : %i", pixel[pixel_index].R);
            //printf("green : %i", pixel[pixel_index].G);
            //printf("blue : %i", pixel[pixel_index].B);
          //}
          //get the middle row of pixels to analyze
          
          int row = scaled_height/2;
          int pixelStart = row*scaled_width;
          int endPixel = pixelStart +scaled_width;
          int counter = 0;
          
          //one row has 320 pixels, set boundaries at 115/125
          for (; pixelStart < endPixel -1; pixelStart++)
          {
            //printf("in for loop");
            pixel_value = (((unsigned int)(pixel[pixelStart].R)) +
                          ((unsigned int)(pixel[pixelStart].G)) +
                          ((unsigned int)(pixel[pixelStart].B))) / 3; // do not worry about rounding
            //pixel[pixel_index].R = pixel_value;
            //pixel[pixel_index].G = pixel_value;
            //pixel[pixel_index].B = pixel_value;
            oneRow[counter] = pixel_value;
            counter = counter +1;
            //printf("\n index:  %i \n", pixel_index);
            
            //printf("red : %i", pixel[pixel_index].R);
            //printf("green : %i", pixel[pixel_index].G);
            //printf("blue : %i", pixel[pixel_index].B);
        }
        //print the array to see whats inside
        int loop;
        //60 after black line
        //printf("\n one Row: ");
        //for(loop = 0; loop < scaled_width-1; loop++)
          //printf("%i ", oneRow[loop]);
        //printf("\n");
        //convert to black and white 
        for(loop = 0; loop < scaled_width-1; loop++) {
          if (oneRow[loop] >=7 )
            oneRow[loop] = 255;
          else
            oneRow[loop] = 0;
        }
        //printf("\n one Row conveted to black and white: ");
        //for(loop = 0; loop < scaled_width-1; loop++)
          //printf("%i ", oneRow[loop]);
        //printf("\n");
        
        //from calibration, first black (0) pixel should be around 118 in oneRow
        //find first black pixel and make decision accordingly
        int firstBlackIndex = -1;
        for(loop = 0; loop < scaled_width-1; loop++){
          if (oneRow[loop] == 0) {
            if (black1 == 1) {
              firstBlackIndex = loop;
              //printf("\nfirst black index : %i \n", firstBlackIndex);
              break;
            }
            else{
              black1 = 1;
            }
          }
          else {
            black1 = 0;
          }
        }
        //keep going straight if its around pixel 115-121
        if ((130<firstBlackIndex) && (firstBlackIndex<190)) {
          straight = 1;
          left = 0;
          right = 0;
        }
        else if (firstBlackIndex <= 130) {
          straight = 0;
          left = 1;
          right = 0;
        }
        else if (firstBlackIndex >= 190) {
          straight = 0;
          left = 0;
          right = 1;
        }
        
        newPicture = 0;
        }
  }
  printf("exiting image processing thread");
  return (void *)0;
}

//camera reading function
void * cameraHandle( void * arg )
{
  Camera = raspicam_wrapper_create();
  //struct timeval current_time;
  ////int seconds = 0;
  //int prevsecs = 0;
  //int prevusecs = 0;
  //int useconds = 0;
  //float usecTimeStamp;
  //float deltaTSecs = 0;
  //int deltaT = 50;
  // create the window
  struct pixel_format_RGB *         RGB_data;
  //initialize camera
  //struct thread_parameter * parameter = (struct thread_parameter *)arg;
  int                               return_value; /* the return value of this program */
  
  if (Camera != NULL)
    {

      if (raspicam_wrapper_open( Camera ))
      {
        // wait a while until camera stabilizes
        printf( "Sleeping for 3 secs\n" );
        sleep( 3 );
        // allocate memory
        data            = (unsigned char *)malloc( raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ) );
        scaled_data     = (unsigned char *)malloc( raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB )/16 );
        RGB_data        = (struct pixel_format_RGB *)data;        // yes, this is aliased data and it is a bad idea
        scaled_RGB_data = (struct pixel_format_RGB *)scaled_data;

        scaled_height = raspicam_wrapper_getHeight( Camera );
        scaled_width  = raspicam_wrapper_getWidth( Camera );
        scaled_height = scaled_height/4;
        scaled_width  = scaled_width/4;
        
        
        while (!done) {
          //get current time stamp 
        //  gettimeofday(&current_time, NULL);
         // prevsecs = seconds;
         // prevusecs = useconds;
         // seconds = current_time.tv_sec;
         // useconds = current_time.tv_usec;
          //usecTimeStamp = useconds / 1000000;
          //float timeStamp = seconds + usecTimeStamp;
              //calculate delta T value
         // if ((seconds - prevsecs) == 0) {
         //   deltaT = useconds - prevusecs;
         //   deltaT = deltaT / 1000; //get value in ms
         // }
         // else if ((seconds-prevsecs) ==1) {
          //  int prevTotal = 1000000-prevusecs;
          //  prevTotal = prevTotal + useconds;
           // deltaT = prevTotal / 1000.0;
          //}
          //deltaTSecs = (float)deltaT/1000.0f;
          //printf( "\ndelta T secs : %f\n", deltaTSecs); //correct
          //printf("\nDeltaT: %i \n", deltaT);
          //printf("seconds : %ld\nmicro seconds : %ld", current_time.tv_sec, current_time.tv_usec);
      
      
        // capture
        raspicam_wrapper_grab( Camera );
        //printf("\ngot picture\n");

        // extract the image in rgb format
        raspicam_wrapper_retrieve( Camera, data, RASPICAM_WRAPPER_FORMAT_IGNORE ); // get camera image
        // scale back the data to a lower and more managable resolution
        scaled_height = raspicam_wrapper_getHeight( Camera );
        scaled_width  = raspicam_wrapper_getWidth( Camera );
        scale_image_data(
            RGB_data,
            scaled_height,
            scaled_width,
            scaled_RGB_data,
            4,
            4 );
            //printf("/n GOT PAST SCALE\n");
        scaled_height = scaled_height/4;
        scaled_width  = scaled_width/4;
        newPicture = 1;
        usleep(20000);
      }
      printf("\nFREEING RESOURCES\n");
      //free resources
      raspicam_wrapper_destroy( Camera );
      free( scaled_data );
      free( data );
    }
      else
      {
        printf( "Error opening camera\n" ); 
        return_value = -1;
      }
    }
    else
    {
      printf( "Unable to allocate camera handle\n" );
      return_value = -1;
    }
    if (return_value == -1)
      printf("return value from camera thread is -1");
  return (void *)0;
}


int get_pressed_key(void)
{
  struct termios  original_attributes;
  struct termios  modified_attributes;
  int             ch;

  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  ch = getchar();

  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  return ch;
} 

//thread function to get user input and set controls
void *ThreadKey( void * arg )
{
  //struct key_thread_parameter *thread_key_parameter = (struct key_thread_parameter *)arg;
  usleep(1000000);
  printf("Homework 8 Program\n");
  printf("Please enter m1 for manual driving mode\n");
  printf("Please enter m2 for self driving, line tracing mode using a camera\n");
  printf("m1 commands: s (stop), w (forward), x (backwards), i (faster), j (slower), a (left), d (right)\n");
  printf("m2 commands: s (stop, w (forward)\n");
  printf("\nHw8>");
  fcntl( STDIN_FILENO, F_SETFL, O_NONBLOCK);
  int ch; 
  do
  {
    switch (get_pressed_key())
    {
      case 'm':
        printf("m");
        //wait for a character
        while ((ch = get_pressed_key()) == EOF);
        if (ch == '1') {
        //set the mode
          m1 = true;
          m2 = false;
          printf("1\n");
          printf("Mode set to m1");
          printf("\nHw8m1>");
        }
        else if (ch == '2') {
        m1 = false;
        m2 = true;
        printf("2\n");
        printf("Mode set to m2");
        printf("\nHw8m2>");
        }
        
      else {
        printf("Invalid command \n ");
        }
    
        break;
      case 'q':
        done = true;
        rightSwitchVar = -1;
         leftSwitchVar = -1;
        break;

      //stop
      case 's':
        stop1 = 1;
        rightSwitchVar =0;
        leftSwitchVar = 0;
        printf("s");
        if (m1) {
          printf("\nHw8m1>");
        }
        if (m2) {
          printf("\nHw8m2>");
        }
        break;
      //forward
      case 'w':

        if (m1) {
          forward1 = 1;
          rightSwitchVar =1;
          leftSwitchVar = 1;
         //printf("in threadkey %i", rightSwitchVar);
          printf("w\nHw8m1>");
        }
        if (m2) {
          forward1 = 1;
          rightSwitchVar =7;
          leftSwitchVar = 7;
          printf("w\nHw8m2>");
        }
        break;
      //backward
      case 'x':
        if (m1) {
          backward1 = 1;
          rightSwitchVar =2;
           leftSwitchVar = 2;
          printf("x\nHw8m1>");
        }
        break;
      //faster
      case 'i':
        if (m1) {
          rightSwitchVar =3;
           leftSwitchVar = 3;
          printf("i\nHw8m1>");
        }
        break;
      //slower 
      case 'j':
      if (m1) {
        rightSwitchVar = 4;
        leftSwitchVar = 4;
        printf("j\nHw8m1>");
      }
        break;
      //left
      case 'a':
      if (m1) {
        rightSwitchVar = 5;
        leftSwitchVar = 5;
        leftCount = leftCount + 1;
        printf("a\nHw8m1>");
      }
        break;
      //right 
      case 'd':
      if (m1) {
        rightSwitchVar = 6;
        leftSwitchVar = 6;
        rightCount = rightCount+1;
        printf("d\nHw8m1>");
      }
        break;
      //no input
      case EOF: 
        break;

      default:
        break;
    }
  } while (!done);
  printf( "key thread exiting\n" );

  return (void *)0;
}



int main( void )
{
 
  pthread_t                       left_handle;
  pthread_t                       right_handle;
  pthread_t                       thread_key_handle;
  pthread_t                       camera_handle;
  pthread_t                       image_process_handle;
  struct thread_parameter         left_parameter;
  struct thread_parameter         right_parameter;
  struct thread_parameter         camera_parameter;
  struct key_thread_parameter     thread_key_parameter;
  struct key_thread_parameter     image_process_parameter;
  volatile struct io_peripherals *io;
  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    // printf( "mem at 0x%8.8X\n", (unsigned long)io );
    enable_pwm_clock( io );
     /* set the pin function to alternate function 0 for GPIO12 */
     /* set the pin function to alternate function 0 for GPIO13 */
     io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
     io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    //set all pins to output 
     io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
     //set IR sensors to input
     //io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
     //io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
    
    /* set the pin function to alternate function 0 for GPIO09 (SPI, MISO) */
    /* set the pin function to alternate function 0 for GPIO10 (SPI, MOSI) */
    /* set the pin function to alternate function 0 for GPIO11 (SPI, SCK) */
    /* set the pin function to output for GPIO08 (SPI, CS) */
    io->gpio.GPFSEL0.field.FSEL9 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL0 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL1 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL0.field.FSEL8 = GPFSEL_OUTPUT;

     /* configure the PWM channels */
     io->pwm.RNG1 = PWM_RANGE;     /* the default value */
     io->pwm.RNG2 = PWM_RANGE;     /* the default value */
     io->pwm.CTL.field.MODE1 = 0;  /* PWM mode */
     io->pwm.CTL.field.MODE2 = 0;  /* PWM mode */
     io->pwm.CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
     io->pwm.CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
     io->pwm.CTL.field.SBIT1 = 0;  /* idle low */
     io->pwm.CTL.field.SBIT2 = 0;  /* idle low */
     io->pwm.CTL.field.POLA1 = 0;  /* non-inverted polarity */
     io->pwm.CTL.field.POLA2 = 0;  /* non-inverted polarity */
     io->pwm.CTL.field.USEF1 = 0;  /* do not use FIFO */
     io->pwm.CTL.field.USEF2 = 0;  /* do not use FIFO */
     io->pwm.CTL.field.MSEN1 = 1;  /* use M/S algorithm */
     io->pwm.CTL.field.MSEN2 = 1;  /* use M/S algorithm */
     io->pwm.CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
     io->pwm.CTL.field.PWEN1 = 1;  /* enable the PWM channel */
     io->pwm.CTL.field.PWEN2 = 1;  /* enable the PWM channel */

    
     //set car to stop 
    GPIO_SET(&(io->gpio), 5);
    GPIO_SET(&(io->gpio), 6);
    GPIO_SET(&(io->gpio), 22);
    GPIO_SET(&(io->gpio), 23);
    io->pwm.DAT1 = 0;
    io->pwm.DAT2 = 0;
    left_parameter.pin1 = 5;
    left_parameter.pin2 = 6;
    right_parameter.pin1 = 22;
    right_parameter.pin2 = 23;
    pthread_create( &left_handle, 0, leftHandle, (void *)&left_parameter );
    pthread_create( &right_handle, 0, rightHandle, (void *)&right_parameter );
    pthread_create( &thread_key_handle, 0, ThreadKey, (void *)&thread_key_parameter );
    pthread_create( &camera_handle, 0, cameraHandle, (void *)&camera_parameter );
    pthread_create( &image_process_handle, 0, imageProcessHandle, (void *)&image_process_parameter );
     //printf("got here");
    pthread_join( left_handle, 0 );
    pthread_join( right_handle, 0 );
    pthread_join( thread_key_handle, 0 );
    pthread_join( camera_handle, 0 );
    pthread_join( image_process_handle, 0 );
    //stop the car before quititng
    GPIO_CLR(&(io->gpio), 5);
    GPIO_CLR(&(io->gpio), 6);
    GPIO_CLR(&(io->gpio), 22);
    GPIO_CLR(&(io->gpio), 23);
    io->pwm.DAT1 = 0;
    io->pwm.DAT2 = 0;
    //set all pins back to input 
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
    //set IR sensors to input
    //io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
    //io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;

    
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}

