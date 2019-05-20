#include <tm4c123gh6pm.h>
#include <stdint.h>

void Time_delay (void){
  for (int i = 0; i<1500000; i++){
  
}
}
void LED_Init(void) { volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x01;       // Activate clock for port A
  delay = SYSCTL_RCGC2_R;             // Allow time for clock to initialize
  GPIO_PORTA_PCTL_R &= ~0x00000F00;   // Regular GPIO
  GPIO_PORTA_AMSEL_R &= ~0xE0;        // Disable analog on PA2
  GPIO_PORTA_DIR_R |= 0xE0;          // Direction PA2 output
  GPIO_PORTA_AFSEL_R &= ~0xE0;        // regular port function
  GPIO_PORTA_DEN_R |= 0xE0;           // Enable digital port
}

int ps = 3; // 1:RED LED 2:GREEN LED 3:YELLOW LED 4:OFF STATE
int ns = 3;
int count = 0;

// Switch check functions
unsigned long Switch1(void) {

  return (GPIO_PORTA_DATA_R&0x04);
}
unsigned long Switch2(void) {

  return (GPIO_PORTA_DATA_R&0x08);
}


void timer0A_delay(void)
{
  TIMER0_CTL_R |= 0x01;        // enable Timer A after initialization 
  
    int i;
    for(i = 0; i < 5; i++) {
      while ((TIMER0_RIS_R & 0x01) != 0x01){      // wait for TimerA timeout flag
      }
     if((Switch2()) && (ps == 1)){
          TIMER1_CTL_R |= 0x01;
          TIMER1_IMR_R = 0x01;
          if (count == 0){
          ns = 2;
          break;
          }
     } 
        
     else if((Switch1()) && (ps != 3)){
           TIMER1_CTL_R |= 0x01;
           TIMER1_IMR_R = 0x01;
           if (count == 0){
          ns = 3;
          break;
          }
        }  
    
      TIMER0_ICR_R = 0x1;      // clear the TimerA timeout flag
    }
    TIMER0_CTL_R |= 0x00;
}

void Switch_Init(void) { volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000001;       // Activate clock for port A
  delay = SYSCTL_RCGC2_R;             // Allow time for clock to initialize
  GPIO_PORTA_AMSEL_R &= ~0x0C;        // Disable analog on PA5
  GPIO_PORTA_PCTL_R &= ~0x00F00000;   // PCTL GPIO on PA5
  GPIO_PORTA_DIR_R &= ~0x0C;          // Direction PA5 input
  GPIO_PORTA_AFSEL_R &= ~0x0C;        // PA5 regular port function
  GPIO_PORTA_DEN_R |= 0x0C;           // Enable PA5 digital port

}


// LED On/Off functions
void LEDG_On(void) {
  GPIO_PORTA_DATA_R |= 0x20;
}
void LEDG_Off(void) {
  GPIO_PORTA_DATA_R &= ~0x20;
}
void LEDY_On(void) {
  GPIO_PORTA_DATA_R |= 0x40;
}
void LEDY_Off(void) {
  GPIO_PORTA_DATA_R &= ~0x40;
}
void LEDR_On(void) {
  GPIO_PORTA_DATA_R |= 0x80;
}
void LEDR_Off(void) {
  GPIO_PORTA_DATA_R &= ~0x80;
}


// Set state functions
void All_Off(void) {
  LEDG_Off();
  LEDY_Off();
  LEDR_Off();
}
void RED(void) {
  LEDG_Off();
  LEDY_Off();
  LEDR_On();
}
void GREEN(void) {
  LEDG_On();
  LEDY_Off();
  LEDR_Off();
}
void YELLOW(void) {
  LEDG_Off();
  LEDY_On();
  LEDR_Off();
}

void Timer1_Handler(void) {
  TIMER1_ICR_R = 0x01;
  ++count;
  if(count == 2) {
    count = 0;
    if((Switch1()) && (ps == 3)){
      ns = 0;
    } else if((Switch2()) && (ps == 1)){
      ns = 2;
    } else if((Switch1()) && (ps != 3)){
      ns = 3;
    }
   
   TIMER1_CTL_R |= 0x00;
   TIMER1_IMR_R = 0x00;
  }

}

int main() {
  Switch_Init();
  LED_Init();
  All_Off();
  SYSCTL_RCGCTIMER_R |= (1<<1);     // enable clock to Timer Block 1
  TIMER1_CTL_R = 0;            //disable Timer before initialization
  TIMER1_CFG_R = 0x00;         //16-bit option 
  TIMER1_TAMR_R = 0x02;        // periodic mode and down-counter
  TIMER1_TAILR_R = 16000000;  // Timer A interval load value register
  TIMER1_ICR_R = 0x1;          // clear the TimerA timeout flag
  //TIMER1_CTL_R |= 0x01;        // enable Timer A after initialization 
  TIMER1_IMR_R = 0x00;
  
  
  SYSCTL_RCGCTIMER_R |= (1<<0);     // enable clock to Timer Block 0
  TIMER0_CTL_R = 0;            //disable Timer before initialization
  TIMER0_CFG_R = 0x00;         //16-bit option 
  TIMER0_TAMR_R = 0x02;        // periodic mode and down-counter
  TIMER0_TAILR_R = 16000000;  // Timer A interval load value register
  TIMER0_ICR_R = 0x1;          // clear the TimerA timeout flag
  //TIMER0_CTL_R |= 0x01;        // enable Timer A after initialization 
  TIMER0_IMR_R = 0x00;
  NVIC_EN0_R |= (1<<21);
  
  while(1) {
    switch(ps) {
      case(0): 
        RED();
        if (Switch1()) {
          TIMER1_CTL_R |= 0x01;
          TIMER1_IMR_R = 0x01;

        } else { // Go Green
          ns = 1;
          timer0A_delay();
          break;
        }
      break;
      
      case(1):
        GREEN();
        if (Switch1()) { // Stop button
          TIMER1_CTL_R |= 0x01;
          TIMER1_IMR_R = 0x01;
          
        } else {
            if (Switch2()) { // Passenger, go yellow
              TIMER1_CTL_R |= 0x01;
              TIMER1_IMR_R = 0x01;
       
              
          } else {
            ns = 0;
            timer0A_delay();
            break; 
          }
        }
      break;
      
      case(2):  
        YELLOW();
        if (Switch1()) {
          TIMER1_CTL_R |= 0x01;
          TIMER1_IMR_R = 0x01;
          
        } else {
          ns = 0;   // Always go red
          timer0A_delay();
        }
      break;
      
      case(3):  
        All_Off();
        if (Switch1()) {
          TIMER1_CTL_R |= 0x01;
          TIMER1_IMR_R = 0x01;
          
        } else {
          ns = 3;
        }
      break;
    }
      ps = ns;
  }
}     