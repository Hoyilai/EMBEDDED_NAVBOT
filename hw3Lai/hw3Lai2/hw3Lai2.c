/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* Homework 3 
* On 4/16  /2021
* By: Hongyi Lai
***************************************************/

/* Homework 3 
 * LED light blinking in sequence program for Raspberry Pi 4 computer
 * RED LED on GPIO 12 w/ 1000 Ohm resistor in series to 3.3V
 * GREEN LED on GPIO 12 w/ 1000 Ohm resistor in series to ground
 * BLUE LED on GPIO 22 w/ 1000 Ohm resistor in series to 3.3v  
 * Orange LED on GPIO 22 w/ 1000 Ohm resistor in series to ground
 */
#include <pthread.h>
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
//variables to be used 
bool  done;

//initialize light states to off
int redGreen = 0;
int blueOrange = 0;

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


//pressed key and change light variables

void *InputChar( void * arg )
{
  if (1)
    
  {

     do
    {
      switch (get_pressed_key())
      {
        
          case 'r': 
      
          if(redGreen == 0) {
             redGreen = 1;
             
          }
          else if(redGreen ==1) {
            redGreen = 0;
            
          }
          
          
          else if(redGreen ==2) {
            redGreen = 3;
            
          }
        
          
          else if(redGreen ==3) {
            redGreen = 2;
          }
          break;
          
          case 'g': 
          
          if(redGreen == 0) {
             redGreen = 2;
          }
          else if(redGreen ==1) {
            redGreen = 3;
          }
          
          
          else if(redGreen ==2) {
            redGreen = 0;
          }
          
          
          else if(redGreen ==3) {
            redGreen = 1;
          }
          break;
          
          case 'b': 
         
          if(blueOrange == 0) {
             blueOrange = 1;
          }
          else if(blueOrange ==1) {
            blueOrange = 0;
          }
        
          
          else if(blueOrange ==2) {
            blueOrange = 3;
          }
          
          
          else if(blueOrange ==3) {
            blueOrange = 2;
          }
          break;
          
          case 'o': 
          if (blueOrange == 0) {
             blueOrange = 2;
          }
          else if (blueOrange ==1) {
            blueOrange = 3;
          }
          
          
          else if (blueOrange ==2) {
            blueOrange = 0;
          }
        
          
          else if (blueOrange ==3) {
            blueOrange = 1;
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

//the light state for pin 12
void *PinTwelveHandler( void * arg )

{
  volatile struct io_peripherals *io;
  io = import_registers();
  if (io != NULL)
    
  {

      while (!done) {
          if (redGreen == 0){
          io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
        }
          
          if (redGreen == 1) {
          io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
          GPIO_CLR(&(io->gpio), 12);
        }
          
          if (redGreen == 2) {
          io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
          GPIO_SET(&(io->gpio), 12);
         }
          
          if (redGreen == 3) {
          while (redGreen == 3) {
          io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
          GPIO_SET(&(io->gpio), 12);
          usleep(1000);
          GPIO_CLR(&(io->gpio), 12);
          usleep(1000);
          }
        }
      }


  }
  else
  {
    ; /* warning message already issued */
  }
  return 0;
}

//function to change light state for pin 22
void *PinTwentyTwoHandler( void * arg )
{
  volatile struct io_peripherals *io;
  io = import_registers();

  if (io != NULL)
    
  {

      while (!done) {
          if (blueOrange == 0){
          io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
        }
          
          if (blueOrange == 1) {
          io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
          GPIO_CLR(&(io->gpio), 22);
        }
          
          if (blueOrange == 2) {
          io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
          GPIO_SET(&(io->gpio), 22);
         }
          
          if (blueOrange == 3) {
          while (blueOrange == 3) {
          io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
          GPIO_SET(&(io->gpio), 22);
          usleep(1000);
          GPIO_CLR(&(io->gpio), 22);
          usleep(1000);
          }
        }
      }


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
  struct thread_parameter         thread12_parameter;
  struct thread_parameter         thread22_parameter;
  struct thread_parameter         thread13_parameter;

  io = import_registers();
  if (io != NULL)
    
  {

    /* turn circuit off for GPIO22 and GPIO12*/
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
  

    
    printf( "\n press 'r' to toggle the red LED\n");
    printf( " press 'g' to toggle the green LED\n");
    printf( " press 'b' to toggle the blue LED\n");
    printf( " press 'o' to toggle the orange LED\n");
    printf( " 'q' character will exit\n");

#if 0
    Thread18( (void *)io );
#else
    thread12_parameter.pin = 12;
    thread12_parameter.gpio = &(io->gpio);
    thread22_parameter.pin = 22;
    thread22_parameter.gpio = &(io->gpio);
    thread13_parameter.pin = 13;
    thread13_parameter.gpio = &(io->gpio);
    pthread_create( &thread12_handle, 0, PinTwelveHandler, (void *)&thread12_parameter );
    pthread_create( &thread22_handle, 0, PinTwentyTwoHandler, (void *)&thread22_parameter );
    pthread_create( &thread13_handle, 0, InputChar, (void *)&thread13_parameter );
    pthread_join( thread12_handle, 0 );
    pthread_join( thread22_handle, 0 );
    pthread_join( thread13_handle, 0 );
#endif


    /* clean the pins */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;

    printf( "\n Key hit is 'q' key, now quiting ... \n");

  }
  else
  {
    ; /* warning message already issued */
  }
  return 0;
}
