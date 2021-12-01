//#include "stm32f10x.h"
#include "STM32F401RE_RCC.h"
#include "STM32F401RE_GPIO.h"
 
void delay(unsigned int nCount);
//GPIO_InitTypeDef GPIO_InitStruct;
int cycle = 0;

void initializeGPIO()
{
    //Turn on clock to GPIOA
    RCC->AHB1ENR.GPIOAEN = 1;
    pinMode(GPIOA, 5, GPIO_OUTPUT);
    pinMode(GPIOA, 6, GPIO_OUTPUT);
    pinMode(GPIOA, 7, GPIO_OUTPUT);
    pinMode(GPIOA, 8, GPIO_OUTPUT);

}

void GPIO_Write(int pin, int val) {
    GPIOA->ODR |= (1 << pin);
}
 
int main(void)
{
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    initializeGPIO();
 
    //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | 
    //    GPIO_Pin_14 | GPIO_Pin_15;
    //GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    //GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    //GPIO_Init(GPIOB, &GPIO_InitStruct);
 
    // One revolution CW using full step mode
    for (cycle = 0; cycle < 512; cycle++)
    {
        GPIO_Write(5, 1);
        delay(5);
        GPIO_Write(6, 1);
        delay(5);
        GPIO_Write(7, 1);
        delay(5);
        GPIO_Write(8, 1);
        delay(5);
    }
 
    delay(1000);
 
    // One revolution CCW using full step mode
    /*for (cycle = 0; cycle < 512; cycle++)
    {
        GPIO_Write(GPIOB, 0x8000);
        delay(5);
        GPIO_Write(GPIOB, 0x4000);
        delay(5);
        GPIO_Write(GPIOB, 0x2000);
        delay(5);
        GPIO_Write(GPIOB, 0x1000);
        delay(5);
    }
    */
    while (1)
    {
    }
}
 
void delay(unsigned int nCount)
{
    unsigned int i, j;
 
    for (i = 0; i < nCount; i++)
        for (j = 0; j < 0x2AFF; j++);
}