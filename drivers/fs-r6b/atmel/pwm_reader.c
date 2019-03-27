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
unsigned int PWM_stat_i[4] = {0, 0, 0, 0};

ISR(PCINT0_vect)
{
  int curr_time = TCNT1;
  // Rising Edge
  if (PWM_stat_i[0] == 0 && PWM_PINB0)
  {
    DEBUG_ON;
    PWM_stat_i[0] = 1;
    PWM_time_i[0] = TCNT1;
  }
  // Falling Edge
  else if (PWM_stat_i[0] == 1 && !PWM_PINB0)
  {
    DEBUG_OFF;
    PWM_stat_i[0] = 0;

    // Overflow prevention
    if (curr_time < PWM_time_i[0])
    {
      PWM_time_d[0] = MAX_COUNT - (PWM_time_i[0] - curr_time);
    }
    else
    {
      PWM_time_d[0] = curr_time - PWM_time_i[0];
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
DDRB &= ~((1<<DDB0));

// Setup pins for pin change interrupts
PCMSK0 |= (1<<PCINT0);
PCICR |= (1<<PCIE0);

// Setup timer
TCCR1B |= TIMR_PRESC;
TCNT1 = 0;

// Enable interrupts
sei();

while (1)
{
  printf("test\n");
  printf("%d \n", PWM_time_d[0]);
  _delay_ms(500);
}

// Never reached
return(0);
}