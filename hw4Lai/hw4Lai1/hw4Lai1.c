/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 4 Program 1
* On 4/19/2022
* By Hongyi Lai
* 
***************************************************/
 
/* Homework 4 Program 1
 * LED light dimming program  in C for 
 * Raspberry Pi 4 computer with 
 * red   LED on GPIO12 (with 1K ohm resistor in series)
 * green LED on GPIO12 (with 1K ohm resistor in series)
 * blue  LED on GPIO13 (with 1K ohm resistor in series)
 * orange LED on GPIO13 (with 1K ohm resistor in series)
 * 
 * Red      LED at GPIO12 goes from     0% light level to 25% light level in    2 seconds,
 * Green   LED at GPIO12 goes from 100% light level to 75% light level in    2 seconds, 
 * Blue      LED at GPIO13 goes from     3% light level to 13% light level in 1.5 seconds, 
 * Orange LED at GPIO13 goes from   97% light level to 87% light level in 1.5 seconds, 
 * lights will dim down in same style
 * hit ctl c to quit
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "enable_pwm_clock.h"
#include <pthread.h>


struct io_peripherals
{
  uint8_t              unused[0x200000];
  struct gpio_register gpio;
};

struct thread_parameter
{
  volatile struct gpio_register * gpio;
  int                             pin;
};


// variables to be used
bool done;

//function to dim red led
void *DimRed(void * arg )
    {
      volatile struct io_peripherals *io;
      io = import_registers();
      int  ONcount, OFFcount;
      //starts from 0
      int level = 0;
      bool up;
      bool down;
      up = true;
      
    while (!done) {
      
      

      while (up) {
      ONcount  =  level;
      OFFcount = 100-level;
      GPIO_SET( &(io->gpio), 12 );  /* ON  LED at GPIO 12 */
      while ((ONcount > 0) && (!done))
      {
        usleep( 130 );
        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 12 );  /* OFF LED at GPIO 12 */

      while ((OFFcount > 0) && (!done))
      {
        
        usleep( 130 );
        OFFcount = OFFcount -1;
      }
      //check if we are done dimming up 
      level = level +1;
      if (level == 25) {
        up = false;
        down = true;
      }
      
    }
      while (down) {
     
      ONcount  =  level;
      OFFcount = 100-level;
      GPIO_SET( &(io->gpio), 12 );  /* ON  LED at GPIO 12 */
      while ((ONcount > 0) && (!done))
      {
        usleep( 130 );
        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 12 );  /* OFF LED at GPIO 12 */

      while ((OFFcount > 0) && (!done))
      {
        usleep( 130 );
        OFFcount = OFFcount -1;
      }
      //check if we are done dimming down 
      level = level -1;
      if (level == 0) {
        up = true;
        down = false;
      }
      
    }
      
    }

    }

//function to dim green LED
void *DimGreen(void * arg )
    {
      volatile struct io_peripherals *io;
      io = import_registers();
      int  ONcount, OFFcount;
      //starts from 100
      int level = 100;
      bool up;
      bool down;
      up = true;
      
      
    while (!done) {
      int loops;
      

      while (up) {
      loops = 2;
      while ((loops>0) && (!done)){
      ONcount  =  level;
      OFFcount = 100-level;
      GPIO_SET( &(io->gpio), 12 );  /* ON  LED at GPIO 12 */
      while ((ONcount > 0) && (!done))
      {
        usleep( 130 );
        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 12 );  /* OFF LED at GPIO 12 */

      while ((OFFcount > 0) && (!done))
      {
        usleep( 130 );
        OFFcount = OFFcount -1;
      }
      loops = loops-1;
    }
    
      //check if we are done dimming down 
      level = level -1;
      if (level == 75) {
        up = true;
        down = false;
      }
      
    }
    
      while (down) {
      loops = 2;
      while ((loops>0) && (!done)){
      ONcount  =  level;
      OFFcount = 100-level;
      GPIO_SET( &(io->gpio), 12 );  /* ON  LED at GPIO 12 */
      
      while ((ONcount > 0) && (!done))
      {
        usleep( 130 );
        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 12 );  /* OFF LED at GPIO 12 */

      while ((OFFcount > 0) && (!done))
      {
        usleep( 130 );
        OFFcount = OFFcount -1;
      }
      loops = loops-1;
    }
      //check if we are done dimming up 
      level = level +1;
      if (level == 100) {
        up = false;
        down = true;
      }
      
    }
      
    }

    }

//funciton to dim blue LED
void *DimBlue(void * arg )
    {
      volatile struct io_peripherals *io;
      io = import_registers();
      int  ONcount, OFFcount;
      //starts from 3
      float level = 3;
      bool up;
      bool down;
      up = true;
      
    while (!done) {
      int loops;
      

      while (up) {
      loops = 15;
      while ((loops>0) && (!done)){
      ONcount  =  level;
      OFFcount = 100-level;
      GPIO_SET( &(io->gpio), 13);  /* ON  LED at GPIO 22 */
      while ((ONcount > 0) && (!done))
      {
        usleep( 250 );
        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 13 );  /* OFF LED at GPIO 22 */

      while ((OFFcount > 0) && (!done))
      {
        usleep( 250 );
        OFFcount = OFFcount -1;
      }
      loops = loops-1;
    }
      //check if we are done dimming up 
      level = level + 1;  
      if (level == 13) {
        up = false;
        down = true;
      }
    }
    
      while (down) {
      loops = 15;
      
      while ((loops > 0) && (!done)){
      ONcount  =  level;
      OFFcount = 100-level;
      GPIO_SET( &(io->gpio), 13 );  /* ON  LED at GPIO 13 */
      while ((ONcount > 0) && (!done))
      {
        usleep( 250 );
        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 13 );  /* OFF LED at GPIO 13 */

      while ((OFFcount > 0) && (!done))
      {
        usleep( 250 );
        OFFcount = OFFcount -1;
      }
      loops = loops-1;
      
    }
      //check if we are done dimming down 
      level = level -1;
      if (level == 3) {
        up = true;
        down = false;
      }
      
    }
      
    }

    }

//function to dim orange LED
void *DimOrange(void * arg )
    {
      volatile struct io_peripherals *io;
      io = import_registers();
      int  ONcount, OFFcount;
      //starts from 97
      float level = 97;
      bool up;
      bool down;
      up = true;
      int loops;

      
    while (!done) {
      

      while (up) {
      loops = 25;
      while ((loops>0) && (!done)) {
      ONcount  =  level;
      OFFcount = 100-level;
      GPIO_SET( &(io->gpio), 13 );  /* ON  LED at GPIO 13*/
      while ((ONcount > 0) && (!done))
      {
        usleep( 250 );

        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 13 );  /* OFF LED at GPIO 13 */

      while ((OFFcount > 0) && (!done))
      {
        usleep( 250 );
        OFFcount = OFFcount -1;
      }
      loops = loops-1;
    }
      //check if we are done dimming down 
      level = level + 1;
      if (level == 87) {
        up = true;
        down = false;
      }
      
    }
      while (down) {
      loops = 25;
      while ((loops>0) && (!done)) {
      ONcount  =  level;
      OFFcount = 100-level;
      GPIO_SET( &(io->gpio), 13 );  /* ON  LED at GPIO 13 */
      while ((ONcount > 0) && (!done))
      {
        usleep( 250 );
        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 13 );  /* OFF LED at GPIO 13 */

      while ((OFFcount > 0) && (!done))
      {
        usleep( 250 );
        OFFcount = OFFcount -1;
      }
      loops = loops-1;
    }
      //check if we are done dimming up 
      level = level - 1;
      if (level == 97) {
        up = false;
        down = true;
      }
      
    }
      
    }

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
//function to get q key
void *InputChar( void * arg )
{
  if (1)
    
  {

     do
    {
      switch (get_pressed_key())
      {
          case 'q':
            done = true;
            break;
        
      }
    } while (!done);

  }
  else
  {
    ; /* warning message already issued */
  }
  return 0;
}



int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t                       thread12_handle;
  pthread_t                       thread22_handle;
  pthread_t                       thread13_handle;
  pthread_t                       thread23_handle;
  pthread_t                       thread25_handle;
  struct thread_parameter         thread12_parameter;
  struct thread_parameter         thread22_parameter;
  struct thread_parameter         thread13_parameter;
  struct thread_parameter         thread23_parameter;
  struct thread_parameter         thread25_parameter;

  io = import_registers();
  if (io != NULL)
  {

    /* set the pin function to OUTPUT for all gpio LED light   */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
    /* set initial output state */
    GPIO_CLR(&(io->gpio), 12); // all lights off to start
    GPIO_CLR(&(io->gpio), 13); //

    printf("\nPress q to quit\n"); 


#if 0
    Thread18( (void *)io );
#else
   //set pin and gpio for each thread
    thread12_parameter.pin = 12;
    thread12_parameter.gpio = &(io->gpio);
    thread13_parameter.pin = 13;
    thread13_parameter.gpio = &(io->gpio);
    thread25_parameter.pin = 25;
    thread25_parameter.gpio = &(io->gpio);
    //create new threads, 3rd parameter is starting routine, 4th parameter is passed to start routine 
    pthread_create( &thread12_handle, 0, DimRed, (void *)&thread12_parameter );
    pthread_create( &thread13_handle, 0, DimGreen, (void *)&thread13_parameter );

    //wait for threads to terminate before returning back 
    pthread_join( thread12_handle, 0 );
    pthread_join( thread13_handle, 0 );

#endif


    /* clean the GPIO pins */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_INPUT;


    printf( "q key hit, now quitting ... \n"); /* this line will not be executed */

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
