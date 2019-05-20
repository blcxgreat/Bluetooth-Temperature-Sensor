#include <tm4c123gh6pm.h>
#include <stdint.h>

#define INITIATE (*((unsigned long*)0x400FE608))
#define GPIOF_DIR (*((unsigned long*)0x40025400))
#define GPIOF_DEN (*((unsigned long*)0x4002551C))
#define GPIOF_DATA (*((unsigned long*)0x400253FC))
#define GPIOF_LOCK_R (*((volatile uint32_t*)0x40025520))
#define GPIOF_CR_R (*((volatile uint32_t*)0x40025524))
#define GPIO_PORTF_PUR_R (*((volatile uint32_t*)0x40025510))

void Timer_Init(void) {
SYSCTL_RCGCTIMER_R |= 0x01;   // Enable the appropriate timer bit
TIMER0_CTL_R = 0x00;
TIMER0_CFG_R = 0x00000000;
TIMER0_TAMR_R = 0x02;
TIMER0_TAILR_R = 0x00F42400;
TIMER0_IMR_R = 0x01;
NVIC_EN0_R |= (1<<19);
}

void PortF_Init(void) {
GPIO_PORTF_IS_R &= ~0x11;
GPIO_PORTF_IBE_R &= ~0x11;
GPIO_PORTF_IEV_R = ~0x11;
GPIO_PORTF_IM_R |= 0x11;
NVIC_EN0_R |= (1<<30);
}

int count = 0;
int flag = 0;

void Timer_Handler(void) {
  TIMER0_ICR_R = 0x01;
  count = 1;
}

void PortF_Int_Handler(void) {
  GPIO_PORTF_ICR_R |= 0x11;
  flag = 1;
}

int main()
{
  INITIATE = 0x20;  //enable Port F GPIO
  GPIOF_DIR = 0x0E;  //Set portF as output
  GPIOF_DEN = 0x1F;  //Enable digital port F
  GPIOF_DATA = 0;    //Clear all port F
  GPIOF_LOCK_R = 0x4C4F434B;   //Unlock the corresponding resister
  GPIOF_CR_R = 0xFF;   //Un-committing the register
  GPIO_PORTF_PUR_R = 0x11;   //Control one register under GPIOF_CR_R
  Timer_Init();
  TIMER0_CTL_R = 0x01; 
  PortF_Init();
  int state = 0;
  
  while (1) {
    int bitwise = GPIOF_DATA & 0x11;   // Check the switches 
    if (flag == 1) { //Override buttons
      flag = 0;
      switch(bitwise){
        case(0x01):
          GPIOF_DATA = 0x02;
          TIMER0_IMR_R = 0x00;
          count = 0;
          break;
        case(0x10):
          TIMER0_IMR_R = 0x01;
          GPIOF_DATA = 0x04;
          break;
      }
    }
    if(count == 1){
      count = 0;
      GPIOF_DATA ^= 0x04;
      GPIOF_DATA &= ~0x02;
    }
  }
  return 0;
}  