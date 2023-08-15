/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 1 
* On 04/13/2022
* By: Hongyi Lai
***************************************************/

/* Homework 1
 * RED LED on GPIO 12 w/ 330 Ohm resistor in series
 * GREEN LED on GPIO 13 w/ 330 Ohm resistor in series
 * BLUE LED on GPIO 22 w/ 330 Ohm resistor in series
 * ORANGE LED on GPIO 23 w/ 330 Ohm resistor in series
 *
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
#include "io_peripherals.h"
#include "enable_pwm_clock.h"


int main( void )
{
  volatile struct io_peripherals *io;

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
    
    
    
    

    /* set initial off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);
    GPIO_CLR(&(io->gpio), 22);
    GPIO_CLR(&(io->gpio), 23);
    
    printf( "hit 'control c' to quit\n");

    while (1)
    {
      //RED on and off
      GPIO_SET( &(io->gpio), 12);

      sleep(1);

      GPIO_CLR( &(io->gpio), 12);

      sleep(1);
      //Green on and off
      GPIO_SET( &(io->gpio), 13);

      sleep(1);

      GPIO_CLR( &(io->gpio), 13);

      sleep(1);
      //Blue on and off
      GPIO_SET( &(io->gpio), 22);

      sleep(1);

      GPIO_CLR( &(io->gpio), 22);

      sleep(1);
      //Orange on and off
      GPIO_SET( &(io->gpio), 23);

      sleep(1);

      GPIO_CLR( &(io->gpio), 23);

      sleep(1);

    }

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
