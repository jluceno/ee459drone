#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "serial_init.c"

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

unsigned int PWM_stat_i1[2] = {0, 0};
unsigned int PWM_time_i1[2] = {0, 0};
unsigned int PWM_time_d1[2] = {0, 0};
unsigned int PWM_max_d1[2] = {0, 0};
unsigned int PWM_min_d1[2] = {UINT32_MAX, UINT32_MAX};
unsigned char PWM_percent1[2] = {0, 0};
unsigned int PWM_stat_i2[2] = {0, 0};
unsigned int PWM_time_i2[2] = {0, 0};
unsigned int PWM_time_d2[2] = {0, 0};
unsigned int PWM_max_d2[2] = {0, 0};
unsigned int PWM_min_d2[2] = {UINT32_MAX, UINT32_MAX};
unsigned char PWM_percent2[2] = {0, 0};


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
    if(*PWM_time_d < *PWM_min_d)
      *PWM_min_d = *PWM_time_d;
    
    if(*PWM_time_d > *PWM_max_d)
      *PWM_max_d = *PWM_time_d;

    // Percentage calculation
    if(*PWM_max_d == *PWM_min_d)
    {
      *res = 0;
    }
    else
    {
      long val = ((long)(*PWM_time_d) -  (long)(*PWM_min_d))*100;
      *res = (unsigned char)(val/(*PWM_max_d - *PWM_min_d));
    }
  }
}

ISR(PCINT1_vect)
{
  int i;
  char PWM_stat1[2] = {PWM_PINC0, PWM_PINC1};

  for(i = 0; i < 2; i++)
  {
    update_values(TCNT1, PWM_stat1[i], &PWM_stat_i1[i], &PWM_time_i1[i],
     &PWM_time_d1[i], &PWM_max_d1[i], &PWM_min_d1[i], &PWM_percent1[i]);
  }
}

ISR(PCINT0_vect)
{
  int i;
  char PWM_stat2[2] = {PWM_PINB0, PWM_PINB1};

  for(i = 0; i < 2; i++)
  {
    update_values(TCNT1, PWM_stat2[i], &PWM_stat_i2[i], &PWM_time_i2[i],
     &PWM_time_d2[i], &PWM_max_d2[i], &PWM_min_d2[i], &PWM_percent2[i]);
  }
}

int main(void)
{
// Setup serial debug
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

// Enable interrupts
sei();

while (1)
{
  printf("%u, %u, %u, %u \n", PWM_percent1[0], PWM_percent1[1], PWM_percent2[0], PWM_percent2[1]);
  _delay_ms(500);
}

// Never reached
return(0);
}