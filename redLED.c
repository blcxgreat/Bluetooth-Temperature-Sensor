#include <tm4c123gh6pm.h>
#include <stdint.h>

#define RED 0x02

int main()
{
  SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;  //enable Port F GPIO
  
  GPIO_PORTF_DIR_R = RED;  //Set portF as output
  GPIO_PORTF_DEN_R = RED;  //Enable digital port F
  GPIO_PORTF_DATA_R = 0;  //Clear all port F
  

  GPIO_PORTF_DATA_R = 0;  //Turn on red LED    

  while (1) {
  }
  return 0;
} 