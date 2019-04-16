#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "serial_init.c"
#include "I2CSlave.h"

#define MAX_COUNT 65535
// Each count takes about 1.0851 microseconds
#define CYCLES_PER_PERIOD 5529
#define CYCLES_0_THROTTLE 922
#define CYCLES_100_THROTTLE 1843
#define NUM_CHAN 4
#define SET_PWM_PC_MASKC (1<<PCINT8) | (1<<PCINT9)
#define SET_PWM_PC_MASKB (1<<PCINT0) | (1<<PCINT1)
#define PWM_DDC_MASK (1<<DDC0) | (1<<DDC1)
#define PWM_DDB_MASK (1<<DDB0) | (1<<DDB1)
#define PWM_PINB0 (PINB & (1<<PINB0))
#define PWM_PINB1 (PINB & (1<<PINB1))
#define PWM_PINC0 (PINC & (1<<PINC0))
#define PWM_PINC1 (PINC & (1<<PINC1))
#define TIMR_PRESC (1<<CS11)
#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define VOLTAGE 4
#define I2C_ADDRESS 24

unsigned int PWM_stat_i1[2] = {0, 0};
unsigned int PWM_time_i1[2] = {0, 0};
unsigned int PWM_time_d1[2] = {0, 0};
unsigned int PWM_max_d1[2] = {1659, 1712};
unsigned int PWM_min_d1[2] = {769, 819};
unsigned char PWM_percent1[2] = {0, 0};
unsigned int PWM_stat_i2[2] = {0, 0};
unsigned int PWM_time_i2[2] = {0, 0};
unsigned int PWM_time_d2[2] = {0, 0};
unsigned int PWM_max_d2[2] = {1805, 1714};
unsigned int PWM_min_d2[2] = {909, 816};
unsigned char PWM_percent2[2] = {0, 0};
unsigned char output;
uint8_t message_reg = 0;


void update_values(int curr_time, char PWM_stat, unsigned int* PWM_stat_i,
 unsigned int* PWM_time_i, unsigned int* PWM_time_d, unsigned int* PWM_max_d,
 unsigned int* PWM_min_d, unsigned char* res)
{
  // Rising Edge
  if (*PWM_stat_i == 0 && PWM_stat)
  {
    *PWM_stat_i = 1;
    *PWM_time_i = curr_time;
  }
  // Falling Edge
  else if (*PWM_stat_i == 1 && !PWM_stat)
  {
    *PWM_stat_i = 0;

    // Overflow prevention
    if (curr_time < *PWM_time_i)
    {
      *PWM_time_d = MAX_COUNT - (*PWM_time_i - curr_time);
    }
    else
    {
      *PWM_time_d = curr_time - *PWM_time_i;
    }

    // Calibration
    //if(*PWM_time_d < *PWM_min_d)
    //  *PWM_min_d = *PWM_time_d;
    
    //if(*PWM_time_d > *PWM_max_d)
    //  *PWM_max_d = *PWM_time_d;

    // Percentage calculation
    if(*PWM_max_d == *PWM_min_d)
    {
      *res = 0;
    }
    else
    {
      long val = ((long)(*PWM_time_d) -  (long)(*PWM_min_d))*100;
      *res = (unsigned char)(val/(*PWM_max_d - *PWM_min_d));
      if(*res > 100)
        *res = 100;
    }
  }
}

ISR(PCINT1_vect)
{
  char PWM_stat1[2] = {PWM_PINC0, PWM_PINC1};

  // CH3
  update_values(TCNT1, PWM_stat1[0], &PWM_stat_i1[0], &PWM_time_i1[0],
    &PWM_time_d1[0], &PWM_max_d1[0], &PWM_min_d1[0], &PWM_percent1[0]);
  // CH4
  update_values(TCNT1, PWM_stat1[1], &PWM_stat_i1[1], &PWM_time_i1[1],
    &PWM_time_d1[1], &PWM_max_d1[1], &PWM_min_d1[1], &PWM_percent1[1]);
}

ISR(PCINT0_vect)
{
  char PWM_stat2[2] = {PWM_PINB0, PWM_PINB1};

  // CH1
  update_values(TCNT1, PWM_stat2[0], &PWM_stat_i2[0], &PWM_time_i2[0],
    &PWM_time_d2[0], &PWM_max_d2[0], &PWM_min_d2[0], &PWM_percent2[0]);
  // CH2
  update_values(TCNT1, PWM_stat2[1], &PWM_stat_i2[1], &PWM_time_i2[1],
    &PWM_time_d2[1], &PWM_max_d2[1], &PWM_min_d2[1], &PWM_percent2[1]);
}

void reg_handler(uint8_t message)
{
  message_reg = message;
  if(message_reg == CH1)
    output = PWM_percent2[0];
  else if(message_reg == CH2)
    output = PWM_percent2[1];
  else if(message_reg == CH3)
    output = PWM_percent1[0];
  else if(message_reg == CH4)
    output = PWM_percent1[1];
  else if(message_reg == VOLTAGE)
    output = 255;
  return;
}

void message_handler()
{
  TWDR = output;
}

int main(void)
{
// Setup serial debug
// Remove this on final build
serial_init (47);
stdout = &uart_output;

// Setup PWM input pins
DDRC &= ~(PWM_DDC_MASK);
DDRB &= ~(PWM_DDB_MASK);

// Setup pins for pin change interrupts
PCMSK1 |= SET_PWM_PC_MASKC;
PCMSK0 |= SET_PWM_PC_MASKB;
PCICR |= ((1<<PCIE1) | (1<<PCIE0));

// Setup timer
TCCR1B |= TIMR_PRESC;
TCNT1 = 0;

// Setup I2C
I2C_setCallbacks(reg_handler, message_handler);
I2C_init(I2C_ADDRESS);

// Enable interrupts
sei();

while (1)
{
  //print mins and maxes
  //printf("%u, %u, %u, %u \n", PWM_max_d2[0], PWM_max_d2[1], PWM_max_d1[0], PWM_max_d1[1]);
  //printf("%u, %u, %u, %u \n", PWM_min_d2[0], PWM_min_d2[1], PWM_min_d1[0], PWM_min_d1[1]);
  
  //print current percentages
  //printf("%u, %u, %u, %u \n", PWM_percent2[0], PWM_percent2[1], PWM_percent1[0], PWM_percent1[1]);
  //_delay_ms(500);
}

// Never reached
return(0);
}