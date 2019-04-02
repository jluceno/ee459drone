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
#define SET_PWM_PC_MASK (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10) | (1<<PCINT11)
#define PWM_DDC_MASK (1<<DDC0) | (1<<DDC1) | (1<<DDC2) | (1<<DDC3)
#define PWM_PINC0 (PINC & (1<<PINC0))
#define PWM_PINC1 (PINC & (1<<PINC1))
#define PWM_PINC2 (PINC & (1<<PINC2))
#define PWM_PINC3 (PINC & (1<<PINC3))
#define DEBUG_ON PORTB |= (1<<PORTB0)
#define DEBUG_OFF PORTB &= ~(1<<PORTB0)
#define TIMR_PRESC (1<<CS11)

unsigned int PWM_time_i[NUM_CHAN] = {0, 0, 0, 0};
unsigned int PWM_time_d[NUM_CHAN] = {0, 0, 0, 0};
unsigned int PWM_max_d[NUM_CHAN] = {0, 0, 0, 0};
unsigned int PWM_min_d[NUM_CHAN] = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
unsigned int PWM_percent[NUM_CHAN] = {0, 0, 0, 0};
unsigned int PWM_stat_i[NUM_CHAN] = {0, 0, 0, 0};

ISR(PCINT1_vect)
{
  int curr_time = TCNT1;
  int i;
  char PWM_stat[NUM_CHAN] = {PWM_PINC0, PWM_PINC1, PWM_PINC2, PWM_PINC3};

  // Check the pins

  for(i = 0; i < NUM_CHAN; i++)
  {
    // Rising Edge
    if (PWM_stat_i[i] == 0 && PWM_stat[i])
    {
      PWM_stat_i[i] = 1;
      PWM_time_i[i] = TCNT1;
    }
    // Falling Edge
    else if (PWM_stat_i[i] == 1 && !PWM_stat[i])
    {
      PWM_stat_i[i] = 0;

      // Overflow prevention
      if (curr_time < PWM_time_i[i])
      {
        PWM_time_d[i] = MAX_COUNT - (PWM_time_i[i] - curr_time);
      }
      else
      {
        PWM_time_d[i] = curr_time - PWM_time_i[i];
      }

      if(PWM_time_d[i] < PWM_min_d[i])
        PWM_min_d[i] = PWM_time_d[i];
      
      if(PWM_time_d[i] > PWM_max_d[i])
        PWM_max_d[i] = PWM_time_d[i];

      // Percentage calculation
      if(PWM_max_d[i] == PWM_min_d[i])
        PWM_percent[i] = 0;
      else
      {
        long val = ((long)PWM_time_d[i] -  (long)PWM_min_d[i])*100;
        PWM_percent[i] = val/(PWM_max_d[i] - PWM_min_d[i]);
      }
    }
  }
}

int main(void)
{

// Setup debug LED
DDRB |= (1<<DDB0);

// Setup serial debug
serial_init (47);
stdout = &uart_output;

// Setup PWM input pins
DDRC &= ~(PWM_DDC_MASK);

// Setup pins for pin change interrupts
PCMSK1 |= SET_PWM_PC_MASK;
PCICR |= (1<<PCIE1);

// Setup timer
TCCR1B |= TIMR_PRESC;
TCNT1 = 0;

// Enable interrupts
sei();

while (1)
{
  printf("%u, %u, %u, %u \n", PWM_percent[0], PWM_percent[1], PWM_percent[2], PWM_percent[3]);
  _delay_ms(500);
}

// Never reached
return(0);
}