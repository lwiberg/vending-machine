#include "STM32F401RE_RCC.h"
#include "STM32F401RE_GPIO.h"
 
#define NUM_STEPS  612  //number of steps in a full rotation
#define MS_DELAY    2   //delay between steps

#define BUTTON_1 10
#define BUTTON_2 1
#define BUTTON_3 4
#define BUTTON_PRESSED 0 //GPIOA, Analog Pin 1
#define DONE 0 //GPIOB, Analog Pin 4
 
void initializeGPIO()
{
    //Set Up Clock
    RCC->CFGR.PPRE2 = 0b000;   //APB High Speed Prescaler = 0
    RCC->CFGR.HPRE  = 0b1001;  //AHP Prescaler = 4
    RCC->AHB1ENR.GPIOAEN = 1;  //turn on clock to GPIOA
    RCC->AHB1ENR.GPIOBEN = 1; 
 
    //Set pins to output mode
    //MOTOR 1
    pinMode(GPIOA, 6, GPIO_OUTPUT);
    pinMode(GPIOA, 7, GPIO_OUTPUT);
    pinMode(GPIOA, 8, GPIO_OUTPUT);
    pinMode(GPIOA, 9, GPIO_OUTPUT);

    //MOTOR 2
    pinMode(GPIOB, 3, GPIO_OUTPUT);
    pinMode(GPIOB, 4, GPIO_OUTPUT);
    pinMode(GPIOB, 5, GPIO_OUTPUT);
    pinMode(GPIOB, 6, GPIO_OUTPUT);

    //MOTOR 3
    pinMode(GPIOA, 5, GPIO_OUTPUT);
    pinMode(GPIOB, 8, GPIO_OUTPUT);
    pinMode(GPIOB, 9, GPIO_OUTPUT);
    pinMode(GPIOB, 10, GPIO_OUTPUT);

    //Buttons
    pinMode(GPIOA, 10, GPIO_INPUT); //Button 1
    pinMode(GPIOA, 1, GPIO_INPUT); //Button 2
    pinMode(GPIOA, 4, GPIO_INPUT); //Button 3

    //Signals to FPGA
    pinMode(GPIOB, 0, GPIO_OUTPUT); //Done
    pinMode(GPIOA, 0, GPIO_OUTPUT); //Button Pressed

}
 
void ms_delay(int ms) {
   while (ms-- > 0) {
      volatile int x=1000;
      while (x-- > 0)
         __asm("nop");}
}

int get_button_press(){
    ms_delay(2); //debounce
    if (digitalRead(GPIOA, BUTTON_1)>0) return 1; 
    if (digitalRead(GPIOA, BUTTON_2)>0) return 2; 
    if (digitalRead(GPIOA, BUTTON_3)>0) return 3; 
    
    return 0;
}
 
void one_step_1(){
    //wave mode
    GPIOA->ODR &= (0x7<<6);
    GPIOA->ODR |= (0x1<<6);
    ms_delay(MS_DELAY);
    GPIOA->ODR &= (0xE<<6);
    GPIOA->ODR |= (0x1<<7);
    ms_delay(MS_DELAY);
    GPIOA->ODR &= (0xD<<6);
    GPIOA->ODR |= (0x1<<8);
    ms_delay(MS_DELAY);
    GPIOA->ODR &= (0xB<<6);
    GPIOA->ODR |= (0x1<<9);
    ms_delay(MS_DELAY);
}

void one_step_2(){
    //wave mode
    GPIOB->ODR &= (0x7<<3);
    GPIOB->ODR |= (0x1<<3);
    ms_delay(MS_DELAY);
    GPIOB->ODR &= (0xE<<3);
    GPIOB->ODR |= (0x1<<4);
    ms_delay(MS_DELAY);
    GPIOB->ODR &= (0xD<<3);
    GPIOB->ODR |= (0x1<<5);
    ms_delay(MS_DELAY);
    GPIOB->ODR &= (0xB<<3);
    GPIOB->ODR |= (0x1<<6);
    ms_delay(MS_DELAY);
}

void one_step_3(){
    //wave mode
    GPIOB->ODR &= (0x7<<7);
    GPIOA->ODR &= (0x7<<5);
    GPIOA->ODR |= (0x1<<5);
    ms_delay(MS_DELAY);
    GPIOB->ODR &= (0xE<<7);
    GPIOA->ODR &= (0xE<<5);
    GPIOB->ODR |= (0x1<<8);
    ms_delay(MS_DELAY);
    GPIOB->ODR &= (0xD<<7);
    GPIOA->ODR &= (0xE<<5);
    GPIOB->ODR |= (0x1<<9);
    ms_delay(MS_DELAY);
    GPIOB->ODR &= (0xB<<7);
    GPIOA->ODR &= (0xE<<5);
    GPIOB->ODR |= (0x1<<10);
    ms_delay(MS_DELAY);
}

void dispense(int motor){
    for(volatile int i; i<NUM_STEPS; i = i+1){
        if (motor == 1) one_step_1();
        if (motor == 2) one_step_2();
        if (motor == 3) one_step_3();
    }
    digitalWrite(GPIOB, DONE, 1);
    ms_delay(100);
}
 
int main(void)
{  
    initializeGPIO();

    while (1){
        digitalWrite(GPIOB, DONE, 0); //reset
        digitalWrite(GPIOA, BUTTON_PRESSED, 0); //reset
        int motor;
        motor = get_button_press();
        if (motor > 0){
            digitalWrite(GPIOA, BUTTON_PRESSED, 1); //send signal to FPGA
            ms_delay(100);
            digitalWrite(GPIOA, BUTTON_PRESSED, 0); //send signal to FPGA
            if (motor == 1) dispense(1); //turn on motor 1
            if (motor == 2) dispense(2); //turn on motor 2
            if (motor == 3) dispense(3); //turn on motor 3
        }
        motor = 0;
    }
}
