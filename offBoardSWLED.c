#include <tm4c123gh6pm.h>
#include <stdint.h>

void LED_Init(void) { volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x01;       // Activate clock for port A
  delay = SYSCTL_RCGC2_R;             // Allow time for clock to initialize
  GPIO_PORTA_PCTL_R &= ~0x00000F00;   // Regular GPIO
  GPIO_PORTA_AMSEL_R &= ~0x04;        // Disable analog on PA2
  GPIO_PORTA_DIR_R |= 0x04;          // Direction PA2 output
  GPIO_PORTA_AFSEL_R &= ~0x04;        // regular port function
  GPIO_PORTA_DEN_R |= 0x04;           // Enable digital port
}

void Switch_Init(void) { volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000001;       // Activate clock for port A
  delay = SYSCTL_RCGC2_R;             // Allow time for clock to initialize
  GPIO_PORTA_AMSEL_R &= ~0x20;        // Disable analog on PA5
  GPIO_PORTA_PCTL_R &= ~0x00F00000;   // PCTL GPIO on PA5
  GPIO_PORTA_DIR_R &= ~0x20;          // Direction PA5 input
  GPIO_PORTA_AFSEL_R &= ~0x20;        // PA5 regular port function
  GPIO_PORTA_DEN_R |= 0x20;           // Enable PA5 digital port
}

void Timer_Init(void) {
SYSCTL_RCGCTIMER_R |= 0x01;   // Enable the appropriate timer bit
TIMER0_CTL_R &= ~0x01;
TIMER0_CFG_R |= 0x00000000;
TIMER0_TAMR_R = 0x02;
TIMER0_TAILR_R = 0xF42400;
TIMER0_CTL_R = 0x01;
}

// Make the LED on
void LED_On(void) {
  GPIO_PORTA_DATA_R |= 0x04;
}

// Make the LED off
void LED_Off(void) {
  GPIO_PORTA_DATA_R &= ~0x04;
}

int main() 
{
  Switch_Init();
  LED_Init();
  LED_Off();    // Initially LED is off
  while(1) {
    int bitwise = GPIO_PORTA_DATA_R & 0x20;
    if (bitwise == 0x20) {
      LED_On();
    } else {
      LED_Off();
    }
  }
}     