/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 4 Program 2
* On 4/19/2022
* By Hongyi Lai
* 
***************************************************/
 
/* Homework 4 Program 2
 * LED light dimming program  in C for 
 * Raspberry Pi 4 computer with 
 * red   LED on GPIO12 (with 1K ohm resistor in series)
 * green LED on GPIO12 (with 1K ohm resistor in series)
 * blue  LED on GPIO13 (with 1K ohm resistor in series)
 * orange LED on GPIO13 (with 1K ohm resistor in series)
 * 
 *
 * toggle light dimming by hitting "r", "g", "b", "o" key 
 * light will dim from 0% to 100% in 3 seconds and vice versa
 * hit q to quit
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
//for each thread
struct thread_parameter
{
  volatile struct gpio_register * gpio;
  int                             pin;
};


// variables to be used
bool done;
int red = 0;
int green = 0;
int blue = 0;
int orange = 0;


//dim lights
void dimLight(int pin, int *color)
    {
      volatile struct io_peripherals *io;
      io = import_registers();
      int  ONcount, OFFcount;
      //starts from 0
      int level=0;
      bool up;
      bool down;
      up = true;
      while (!done) {
        int loops;

        while (((*color)==1) && (!done)) {
      
     
          //light dim up
          while (up) {
            loops = 2;
            while ((loops>0) && (!done)){
            ONcount  =  level;
            OFFcount = 100-level;
            GPIO_SET( &(io->gpio), pin );  /* ON  LED */
            while ((ONcount > 0) && (!done) && ((*color) ==1))
            {
              usleep( 80 );
              ONcount = ONcount -1;
            }     

        GPIO_CLR( &(io->gpio), pin );  /* OFF LED  */

        while ((OFFcount > 0) && (!done) && ((*color)==1))
          {
            usleep( 80 );
            OFFcount = OFFcount -1;
          }
          loops = loops-1;
        }
          //check done dimming up 
        level = level +1;
        if (level == 100) {
          up = false;
          down = true;
        }

        }
      //dim down
        while (down) {
        loops = 2;
        while ((loops>0) && (!done)){
        ONcount  =  level;
        OFFcount = 100-level;
        GPIO_SET( &(io->gpio), pin );  /* ON  LED  */
        while ((ONcount > 0) && (!done) && ((*color) ==1))
          {
          usleep( 80 );
          ONcount = ONcount -1;
          }
        GPIO_CLR( &(io->gpio), pin );  /* OFF LED  */
        while ((OFFcount > 0) && (!done) && ((*color) == 1))
        {
          usleep( 80 );
          OFFcount = OFFcount -1;
        }
        loops = loops-1;
      }
        //check if we are done dimming down 
        level = level -1;
        if (level == 0) {
          up = true;
          down = false;
        }
      
      }
      
      }
      //loop to hold the light at the same level while paused
          while (((*color)==0) && (!done)) {

                ONcount  =  level;
                OFFcount = 100-level;

                GPIO_SET( &(io->gpio), pin );  /* ON  LED  */
                while ((ONcount > 0) && (!done) && ((*color) == 0))
                {
                  usleep( 70 );
                  ONcount = ONcount -1;
                  }
                GPIO_CLR( &(io->gpio), pin );  /* OFF LED */

                while ((OFFcount > 0) && (!done) && ((*color) == 0))
                  {
                  usleep( 70 );
                  OFFcount = OFFcount -1;
                }

        }

  }
}


void *handleLight(void * arg )
    {
      volatile struct io_peripherals *io;
      io = import_registers();
      int  ONcount, OFFcount;
      struct thread_parameter * parameter = (struct thread_parameter *)arg;
      //starts from 0
      bool up;
      bool down;
      up = true;
      int *color;
      //set the color accordingly
      if (parameter->pin == 12) {
        color = &red;
        dimLight(parameter->pin, color);
      }
      else if (parameter->pin == 12) {
        color = &green;
       dimLight(parameter->pin, color);
      }

      else if (parameter->pin == 13) {
        color = &blue;
        dimLight(parameter->pin, color);
      }

      else if (parameter->pin == 13) {
        color = &orange;
        dimLight(parameter->pin, color);
      }
    
  }



//function to get user input
    
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

//function to change light variables

void *InputChar( void * arg )
{
  if (1)
  {
    
     do
    {
      switch (get_pressed_key())
      {
          case 'r': 
      
          if(red == 0) {
            
             red = 1;
          }
          else if(red ==1) {
            red = 0;
          }
          break;
          
          case 'g': 
          if(green == 0) {
             green = 1;
          }
          else if(green ==1) {
            green = 0;
          }
          break;
          
          case 'b': 
          if(blue == 0) {
             blue = 1;
          }
          else if(blue ==1) {
            blue = 0;
          }
          break;
          
          case 'o': 
          if (orange == 0) {
             orange = 1;
          }
          else if (orange ==1) {
            orange = 0;
          }
          break;
          
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
  pthread_t                       thread13_handle;
  struct thread_parameter         thread12_parameter;
  struct thread_parameter         thread13_parameter;
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
    
    
    printf( "\n press 'r' to toggle the red LED\n");
    printf( " press 'g' to toggle the green LED\n");
    printf( " press 'b' to toggle the blue LED\n");
    printf( " press 'o' to toggle the orange LED\n");
    printf( " 'q' character will exit\n");


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
    pthread_create( &thread12_handle, 0, handleLight, (void *)&thread12_parameter );
    pthread_create( &thread13_handle, 0, handleLight, (void *)&thread13_parameter );

  
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
