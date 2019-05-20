// Lab3 partA program

#include <tm4c123gh6pm.h>
#include <stdint.h>

double temp;
void delay(int second){
  for(int i = 0; i<second; i++){
  }
}

void Timer0_Init(unsigned long count) {
//SYSCTL_RCGCTIMER_R |= 0x01;   // Enable the appropriate timer bit
SYSCTL_RCGCWTIMER_R |= 1;    /* enable clock to WTimer Block 0 */
WTIMER0_CTL_R = 0;           /* disable WTimer before initialization */
WTIMER0_CFG_R = 0x04;        /* 32-bit option */
WTIMER0_TAMR_R = 0x02;       /* periodic mode and down-counter */
WTIMER0_TAILR_R = count;  /* WTimer A interval load value reg (1 s) */
WTIMER0_CTL_R |= 0x20;       /* timer triggers ADC */
WTIMER0_CTL_R |= 0x01;       /* enable WTimer A after initialization */
}

void PortF_Init(void) {
SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;  //enable Port F GPIO
while((SYSCTL_PRGPIO_R&0x0020) == 0){};/* ready? */
GPIO_PORTF_DIR_R = 0x0E;  //Set portF as output
GPIO_PORTF_DEN_R = 0x1F;  //Enable digital port F
GPIO_PORTF_DATA_R = 0;  //Clear all port F
GPIO_PORTF_LOCK_R = 0x4C4F434B;   //Unlock the corresponding resister
GPIO_PORTF_CR_R = 0xFF;   //Un-committing the register
GPIO_PORTF_PUR_R = 0x11;   //Control one register under GPIOF_CR_R
}

void PLL_Init(uint32_t freq) {
/* 1) configure the system to use RCC2 for advanced features
    such as 400 MHz PLL and non-integer System Clock Divisor */
SYSCTL_RCC2_R |= 0x80000000;
/* 2) bypass PLL while initializing */
SYSCTL_RCC2_R |= (1<<11);
/* 3) select the crystal value and oscillator source */
SYSCTL_RCC_R = (SYSCTL_RCC_R&~0x7C0)+0x540;       /*selecting crystal value and oscillator source*/
SYSCTL_RCC2_R &= ~0x70;			/* configure for main oscillator*/
/* 4) activate PLL by clearing PWRDN */
SYSCTL_RCC2_R &= ~(1<<13);
/* 5) set the desired system divider and the system divider least significant bit */
SYSCTL_RCC2_R |= (1<<30);  	/* use 400 MHz PLL */
SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000) /* clear system clock divider field */
                + (freq<<22)/*(0x63<<22)    ((0x04<<22) == (0x2<<23))*/;      	    /* configure for 4MHz or 80 MHz clock */ 
/* 6) wait for the PLL to lock by polling PLLLRIS */
while((SYSCTL_RIS_R&0x40)==0){};
/* 7) enable use of PLL by clearing BYPASS */
SYSCTL_RCC2_R &= ~(1<<11);
}

void ADC_Init(void) { //volatile unsigned long delay;
SYSCTL_RCGCADC_R |= 1;       /* enable clock to ADC0 */
//delay = SYSCTL_RCGCADC_R;             /* Allow time for clock to initialize*/
//delay(1000000);
ADC0_ACTSS_R &= ~8;          /* disable SS3 during configuration */
ADC0_EMUX_R &= ~0xF000;
ADC0_EMUX_R |= 0x5000;       /* timer trigger conversion seq 0 */
ADC0_SSMUX3_R = 0;           /* get input from channel 0 */
ADC0_SSCTL3_R |= 0x0E;       /* take chip temperature, set flag at 1st sample */
ADC0_IM_R = (1<<3);         /* mask the interrupt */
NVIC_PRI4_R |= 0;          /* set the interrupt priority */
NVIC_EN0_R |= (1<<17);
ADC0_ISC_R = (1<<3);       /* enable the interrupt */
ADC0_ACTSS_R |= 8;           /* enable ADC0 sequencer 3 */

}

void ADC0_Handler(void) {
  temp = (147.5 - (247.5 * ADC0_SSFIFO3_R) / 4096);
  ADC0_ISC_R = 0x8;                /* clear completion flag */
}

int main()
{
  PLL_Init(4);
  PortF_Init();
  
  ADC_Init();
  Timer0_Init(80000000);
  
  while (1) {
    int bitwise = GPIO_PORTF_DATA_R & 0x11;   // Check the switches
    if(bitwise == 0x10){       // PF1
      PLL_Init(4);
      Timer0_Init(80000000);
    } else if(bitwise == 0x01) {      // Base case: PF0
      PLL_Init(99);
      Timer0_Init(4000000);
    }
    if(temp < 17){
      GPIO_PORTF_DATA_R = 0x02;            // red
    } else if(temp < 19){
      GPIO_PORTF_DATA_R = 0x04;           // blue
    } else if(temp < 21){
      GPIO_PORTF_DATA_R = 0x06;          // violet
    } else if(temp < 23){
      GPIO_PORTF_DATA_R = 0x08;          // green
    } else if(temp < 25){
      GPIO_PORTF_DATA_R = 0x0A;          // yellow 
    } else if(temp < 27){
      GPIO_PORTF_DATA_R = 0x0C;          // cyan 
    } else {
      GPIO_PORTF_DATA_R = 0x0E;          // white 
    }

  }
  return 0;
}  


