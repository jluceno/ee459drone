#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "serial_init.c"

#define NUM_CHAN 1
#define MAX_COUNT 65535
// Each count takes about 1.0851 microseconds
#define CYCLES_PER_PERIOD 5529
#define CYCLES_0_THROTTLE 922
#define CYCLES_100_THROTTLE 1843
#define SET_PWM_PC_MASK (1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2) | (1<<PCINT3)
#define PWM_DDB_MASK (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3)
#define PWM_PINB0 (PINB & (1<<PINB0))
#define PWM_PINB1 (PINB & (1<<PINB1))
#define PWM_PINB2 (PINB & (1<<PINB2))
#define PWM_PINB3 (PINB & (1<<PINB3))
#define DEBUG_ON PORTC |= (1<<PORTC0)
#define DEBUG_OFF PORTC &= ~(1<<PORTC0)
#define TIMR_PRESC (1<<CS11)

unsigned int PWM_time_i[4] = {0, 0, 0, 0};
unsigned int PWM_time_d[4] = {0, 0, 0, 0};
unsigned int PWM_percent[4];
unsigned char PWM_stat_i[4];

ISR(PCINT0_vect)
{
  unsigned int curr_time = TCNT1;
  unsigned char PWM_curr_stat[NUM_CHAN] = {PWM_PINB0};
  int i;

  for (i = 0; i < NUM_CHAN; i++)
  {
    //Rising edge detected
    if (PWM_stat_i[i] == 0 && PWM_curr_stat[i])
    {
      PWM_time_i[i] = curr_time;
      PWM_stat_i[i] = 1;
      continue;
    }

    //Falling edge detected
    else if (PWM_stat_i[i] == 1 && !(PWM_curr_stat[i]))
    {
      PWM_stat_i[i] == 0;
      // Overflow case
      if (PWM_time_i[i] > curr_time)
      {
        PWM_time_d[i] = PWM_time_i[i] - curr_time;
        PWM_time_d[i] = MAX_COUNT - PWM_time_d[i];
      }
      // Normal case
      else
      {
        PWM_time_d[i] = curr_time - PWM_time_i[i];
      }

      // Calculate throttle percentage
      PWM_percent[i] = ((PWM_time_d[i] - CYCLES_0_THROTTLE)*100) /
        (CYCLES_100_THROTTLE - CYCLES_0_THROTTLE);
      if (PWM_percent[i] > 100)
        PWM_percent[i] = 100;

      int j;
      printf("vals: ");
      for (j = 0; j < NUM_CHAN; j++)
        printf("%u ", PWM_percent[j]);
      printf("\n");
    }
  }
}

int main(void)
{

// Setup debug LED
DDRC |= (1<<DDC0);

// Setup serial debug
serial_init (47);
stdout = &uart_output;

// Setup PWM input pins
DDRB &= ~(PWM_DDB_MASK);

// Setup pins PB0-PB3 for pin change interrupts
PCMSK0 |= SET_PWM_PC_MASK;
PCICR |= (1<<PCIE0);

// Setup timer
TCCR1B |= TIMR_PRESC;
TCNT1 = 0;

// Enable interrupts
sei();

while (1)
{

}

// Never reached
return(0);
}