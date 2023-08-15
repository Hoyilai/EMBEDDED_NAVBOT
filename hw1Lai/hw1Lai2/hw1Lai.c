/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 1 Program 1
* Revision V1.1
* On 04/13/2022
* By: Hongyi Lai
***************************************************/



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
#include "io_peripherals.h"
#include "enable_pwm_clock.h"

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


int main( void )
{
  volatile struct io_peripherals *io;
  bool  done;
  
  io = import_registers();
  if (io != NULL)
    
  {

    /* GPIO12 - red LED light   */
    /* GPIO13 - green LED light */
    /* GPIO22 - blue LED light */
    /* GPIO23 - orange LED light */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;

    /* off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);
    GPIO_CLR(&(io->gpio), 22);
    GPIO_CLR(&(io->gpio), 23);
    
    printf( "hit 'ctl c' to quit\n");

    printf( "\n press 'r' to toggle the red LED\n");
    printf( " press 'g' to toggle the green LED\n");
    printf( " press 'b' to toggle the blue LED\n");
    printf( " press 'o' to toggle the orange LED\n");
    printf( " any other character will exit\n");
    
    do
    {
      switch (get_pressed_key())
      {
        case 'r':
          if (GPIO_READ(&(io->gpio), 12) == 0)
          {
            GPIO_SET( &(io->gpio), 12);
          }
          else
          {
            GPIO_CLR( &(io->gpio), 12);
          }
          break;

        case 'g':
          if (GPIO_READ(&(io->gpio), 13) == 0)
          {
            GPIO_SET( &(io->gpio), 13);
          }
          else
          {
            GPIO_CLR( &(io->gpio), 13);
          }
          break;
          
      case 'b':
          if (GPIO_READ(&(io->gpio), 22) == 0)
          {
            GPIO_SET( &(io->gpio), 22);
          }
          else
          {
            GPIO_CLR( &(io->gpio), 22);
          }
          break;
      
      case 'o':
          if (GPIO_READ(&(io->gpio), 23) == 0)
          {
            GPIO_SET( &(io->gpio), 23);
          }
          else
          {
            GPIO_CLR( &(io->gpio), 23);
          }
          break;

        default:
          done = true;
          break;
      }
    } while (!done);

    /* clean the GPIO pins */


    printf( "\n Key hit is not 'r' or 'g' key, now quiting ... \n");

  }
  else
  {
  }

  return 0;
}
