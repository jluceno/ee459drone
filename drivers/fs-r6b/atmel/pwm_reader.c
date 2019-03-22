#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define SET_PWM_PC_MASK (1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2) | (1<<PCINT3)
#define PWM_DDB_MASK (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3)
#define PWM_PINB0 (PINB & (1<<PINB0))
#define DEBUG_ON PORTC |= (1<<PORTC0)
#define DEBUG_OFF PORTC &= ~(1<<PORTC0)

ISR(PCINT0_vect)
{
  if (PWM_PINB0)
  {
    DEBUG_ON;
  }
  else
  {
    DEBUG_OFF;
  }
}

int main(void)
{

// Setup debug LED
DDRC |= (1<<DDC0);

// Setup PWM input pins
DDRB &= ~(1<<DDB0);

// Setup pins PB0-PB3 for pin change interrupts
PCMSK0 |= SET_PWM_PC_MASK;
PCICR |= (1<<PCIE0);

// Enable interrupts
sei();

while (1)
{
  
}

// Never reached
return(0);
}