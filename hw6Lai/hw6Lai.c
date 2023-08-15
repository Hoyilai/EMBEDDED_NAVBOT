/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 6
* Revision V1.1
* On 4/28/2022
* By Hongyi Lai
* 
***************************************************/

/* Homework 6
 * Car project with two modes
 * m1 - Manual Control Mode
 * m2 - Line Tracing Self-Driving Mode
 */



#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"

struct pause_flag{
    pthread_mutex_t lock;
    bool  pause;
  };
  
struct done_flag{
    pthread_mutex_t lock;
    bool  done;
  };
  
struct thread_parameter{
    volatile struct gpio_register * gpio;
    volatile struct pwm_register * pwm;
    int pin1;
    int pin2;
    struct pause_flag * pause;
    struct done_flag * done; 
  };

struct key_thread_parameter{
    struct done_flag * done;
    struct pause_flag * pause1;
    struct pause_flag * pause2;
    struct pause_flag * pause3;
    struct pause_flag * pause4;
  };

 //set the right and left variale to check cases
int rightswitch = -1;
int leftswitch = -1; 
//set the global variables 
int forward = -1;
int backward = -1;
int stop = -1;
bool done = false;
//check 2 modes status
bool m1 = false;
bool m2 = false;
//check sensor status
bool rightsensor = false;
bool leftsensor = false;
//set the PWM range as 100
#define PWM_RANGE 100

int dutyLevelLeft = 0;
int dutyLevelRight = 0;
int dutyLevelLefton = 0;
int dutyLevelRighton = 0;
int leftCount;
int rightCount;

void *leftmaster( void * arg )
{
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  int pin1 = parameter->pin1;
  int pin2 = parameter->pin2;
  volatile struct io_peripherals *io;
  int currentDuty;
  io = import_registers();

do
  {
    
    switch (leftswitch)
    {
      //stop 
      case 0:
      if ((backward == 1) || (forward ==1)) {
        backward = 0;
        forward = 0;
        //set pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      stop = 1;
      //set pins to stop
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);
      leftswitch = -1;
      break;
      //forward
      case 1: 
      if ((backward == 1) || (stop ==1))  {
        backward = 0;
        stop = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      forward = 1;
      //set pins to forward 
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);  

      leftswitch = -1;
      break;

      //backward
      case 2:
      if ((forward == 1) || (stop ==1)) {
        forward =0;
        stop = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      backward = 1;
      //set pins to backwards 
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_SET(&(io->gpio), pin2);
      
      leftswitch = -1;
      break;
      
      
      //faster
      case 3:
      //get current duty level
      currentDuty = dutyLevelLeft;
      //add 5 which can be added 20 times
      currentDuty = currentDuty + 5;
      if (currentDuty >=100){
        currentDuty = 100;
      }
      //set PWM to this level
      io->pwm.DAT2 = currentDuty;
      //save updated level 
      dutyLevelLeft = currentDuty;
      leftswitch = -1;
      break;
      
      
      //slower
      case 4:
      //get current duty level
      currentDuty = dutyLevelLeft;
      //subtract 5 which can be added 20 times
      currentDuty = currentDuty - 5;
      if (currentDuty <=0){
        currentDuty = 0;
      }
      //set PWM to this level
      io->pwm.DAT2 = currentDuty;
      //save updated level to global
      dutyLevelLeft = currentDuty;
      leftswitch = -1;
      break;

      //left
      case 5:
      //decrease left motor 
       dutyLevelLefton = dutyLevelLeft;
      while ((leftCount > 0) && (!done)){
        //change 15 degree
        io->pwm.DAT2 = 40 ;
        // sleep for .5 secs
        usleep(500000);
        leftCount = leftCount-1;
      }
      io->pwm.DAT2 = dutyLevelLefton;
      leftswitch = -1;
      break;
      //right
      case 6:
      leftswitch = -1;
      break;

      case 7:
        if((backward == 1) || (stop == 1)){
          backward = 0;
          stop = 0;
          //set pins to pause
          GPIO_SET(&(io->gpio), pin1);    
          GPIO_SET(&(io->gpio), pin2);
          usleep(500000);         
        }
        forward = 1;
        GPIO_SET(&(io->gpio), pin1);
        GPIO_CLR(&(io->gpio), pin2);      
        //set duty level as 50
        io->pwm.DAT2= 25;
        //check black line 
        while ((!done) && (leftswitch ==7)) {
        while ((GPIO_READ(&(io->gpio), 24) == 0) && (!done) && (leftswitch ==7) ){
            leftsensor = false;
        }
        while ((GPIO_READ(&(io->gpio), 24) !=0) && (!done) && (leftswitch ==7) ){
          leftsensor = true;
          GPIO_CLR(&(io->gpio), pin1);
          GPIO_SET(&(io->gpio), pin2); 
          io->pwm.DAT2 = 60;
          io->pwm.DAT1 = 35;
          usleep(500000);
        }
        GPIO_SET(&(io->gpio), pin1);
        GPIO_CLR(&(io->gpio), pin2);
        io->pwm.DAT2 = 25;
        io->pwm.DAT1 = 25;
        }    
      break;
 
      case -1:
      break;

      default:
        break;
    }
  } while (!done);
   
  return (void *)0;
}

//right motor control function
void *rightmaster( void * arg )
{
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  int pin1 = parameter->pin1;
  int pin2 = parameter->pin2;
  volatile struct io_peripherals *io;
  int currentDuty;
  io = import_registers();
while (!done)
  {
    switch (rightswitch)
    {

      //stop 
      case 0:
        if ((backward == 1) || (forward ==1)) {
        backward = 0;
        forward = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      stop = 1;
      //set pins to stop 
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);
      
      rightswitch = -1;
      break;

      //forward
      case 1: 
        if ((backward == 1) || (stop ==1))  {
        backward = 0;
        stop = 0;
        //set both pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      forward = 1;
      //set pins to forward 
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2); 

      rightswitch = -1;
      break;
 
      //backward
      case 2:
        if ((forward == 1) || (stop ==1)) {
        forward =0;
        stop = 0;
        //set pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      backward = 1;
      //set pins to backwards 
      GPIO_CLR(&(io->gpio), pin1);
      GPIO_SET(&(io->gpio), pin2);
            
      rightswitch = -1;
      break;

      //faster
      case 3:
      //get current duty level
      currentDuty = dutyLevelRight;
      //add 5 which can be added 20 times
      currentDuty = currentDuty + 5;
      if (currentDuty >=100){
        currentDuty = 100;
      }
      //set PWM to this level
      io->pwm.DAT1 = currentDuty;
      //save updated level to global
      dutyLevelRight = currentDuty;
      rightswitch = -1;
      break;
      //slower
      case 4:
      //get current duty level
      currentDuty = dutyLevelRight;
      //add 15 (one step)
      currentDuty = currentDuty - 5;
      if (currentDuty <=0){
        currentDuty = 0;
      }
      //set PWM to this level
      io->pwm.DAT1 = currentDuty;
      //save updated level to global
      dutyLevelRight = currentDuty;
      rightswitch = -1;
      break;
      //left
      case 5:
      rightswitch = -1;
      break;
      //right
      case 6:
      //decrease right motor 
      dutyLevelRighton = dutyLevelRight;
      while ((rightCount > 0) && (!done)){
        //decrease to 40
        io->pwm.DAT1 = 40;
        // sleep for .5 secs
        usleep(500000);
        rightCount = rightCount-1;
      }
      //go back to previous duty level 
      io->pwm.DAT1= dutyLevelRighton;
      rightswitch = -1;
      break;

      //line tracing
      case 7:
        if ((backward == 1) || (stop ==1))  {
        backward = 0;
        stop = 0;
        //set pins to pause
        GPIO_SET(&(io->gpio), pin1);  
        GPIO_SET(&(io->gpio), pin2);
        usleep(500000);
      }
      forward = 1;
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2);  
      //set duty level to 50
      io->pwm.DAT1= 25;
      //check black line 
      while ((!done) && (rightswitch ==7)) {
      while ((GPIO_READ(&(io->gpio), 25) == 0) && (!done)&& (rightswitch ==7)) {
        rightsensor = false;
      }
      while ((GPIO_READ(&(io->gpio), 25) !=0) && (!done)&& (rightswitch ==7)) {
        rightsensor = true;
        GPIO_CLR(&(io->gpio), pin1);
        GPIO_SET(&(io->gpio), pin2); 
        io->pwm.DAT2 = 35;
        io->pwm.DAT1 = 60;
        usleep(500000);
      }
      GPIO_SET(&(io->gpio), pin1);
      GPIO_CLR(&(io->gpio), pin2); 
      io->pwm.DAT2 = 25;
      io->pwm.DAT1= 25;
      }
      break;

      case -1:
      break;

      default:
        break;
    }
  } 

  return (void *)0;
}

int input_key(void){
    struct termios original_attributes;
    struct termios modified_attributes;
    int character;

    tcgetattr( STDIN_FILENO, &original_attributes );
    modified_attributes = original_attributes;
    modified_attributes.c_lflag &= ~(ICANON | ECHO);
    modified_attributes.c_cc[VMIN] = 1;
    modified_attributes.c_cc[VTIME] = 0;
    tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

    character = getchar();

    tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

    return character;
  }

void *Key(void * arg){
    struct key_thread_parameter *thread_key_parameter = (struct key_thread_parameter *)arg;
    
    fcntl( STDIN_FILENO, F_SETFL, O_NONBLOCK);
    
    int character;
    
    printf("Please choose the mode:\n");
    printf("m1 : manual control mode\n");
    printf("m2 : line tracing self-driving mode\n");
    printf("****************************************\n");
    while(!done){
          switch(input_key()){
            
              case 'm':
              //Identify the mode with input character
                while((character = input_key()) == EOF);
                  if( character == '1'){
                    m1 = true;
                    m2 = false;
                    printf("Entering m1 mode.\n");
                    }
                  else if( character == '2' ){
                    m1 = false;
                    m2 = true;
                    printf("Entering m2 mode.\n");                    
                    }
                  else{
                    printf("Invalid character.\n");
                    }
                
                break;
            
              case 'q':
                printf("\nQuiting....");
                done = true;
                rightswitch = -1;
                leftswitch = -1;
                
                break;
                
              case 's':
              if( m1 == true){
                stop = 1;
                rightswitch = 0;
                leftswitch = 0;
                printf("Stopping m1 mode\n");
              }
              else if( m2 == true ){
                stop = 1;
                rightswitch = 0;
                leftswitch = 0;
                printf("Stopping m2 mode\n");
                }
                
                break;
                
              case 'w':
              if( m1 == true ){
                forward = 1;
                rightswitch = 1;
                leftswitch = 1;
                printf(" m1 mode: Forwarding\n");
                
              }
              else if( m2 == true ){
                forward = 1;
                rightswitch = 7;
                leftswitch = 7;
                printf(" m2 mode: Forwarding\n");
                }
                break;
                
              case 'x':
                backward = 1;
                rightswitch = 2;
                leftswitch = 2;
                printf("Backwarding\n");
                break;
                
              case 'i':
                rightswitch = 3;
                leftswitch = 3;
                printf("Speeding up!\n");
                break;
              
              case 'j':
                rightswitch = 4;
                leftswitch = 4;
                printf("Slowing down!\n");
                break;
                
              case 'a':
                rightswitch = 5;
                leftswitch = 5;
                leftCount = leftCount + 1;
                printf("Turning left\n");
                break;
                
              case 'd':
                rightswitch = 6;
                leftswitch = 6;
                rightCount = rightCount + 1;
                printf("Turning right\n");
                break;
                
              default:
                break;
              
            }
      }
      return (void *) 0;
  }

int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t                       left_handle;
  pthread_t                       right_handle;
  pthread_t                       thread_key_handle;
  struct thread_parameter         left_parameter;
  struct thread_parameter         right_parameter;
  struct key_thread_parameter     thread_key_parameter; 
  io = import_registers();
  if (io != NULL)
    {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to INPUT to start (turn circuit off) for GPIO12 and GPIO13*/
     io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
     io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    //configure all the outputs
     io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
     io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
     

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
     
     //turn on the car with 4 pins
     GPIO_CLR(&(io->gpio), 5);
     GPIO_CLR(&(io->gpio), 6);
     GPIO_CLR(&(io->gpio), 22);
     GPIO_CLR(&(io->gpio), 23);
     io->pwm.DAT1 = 0;
     io->pwm.DAT2 = 0;
     left_parameter.pin1 = 5;
     left_parameter.pin2 = 6;
     right_parameter.pin1 = 22;
     right_parameter.pin2 = 23;
    //load the programs
     pthread_create( &left_handle, 0, leftmaster, (void *)&left_parameter );
     pthread_create( &right_handle, 0, rightmaster, (void *)&right_parameter );
     pthread_create( &thread_key_handle, 0, Key, (void *)&thread_key_parameter ); 
        
     pthread_join( left_handle, 0 );
     pthread_join( right_handle, 0 );
     pthread_join( thread_key_handle, 0 );
     
     printf("\nQuited!\n *********************");
  }
  else
  {
    ; /* warning message already issued */
  }
  return 0;
}
