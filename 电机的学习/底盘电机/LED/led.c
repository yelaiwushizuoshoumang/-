 #include "led.h"
#include "stm32f4xx.h"

void led_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOA, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;//|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
  
   	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);


    GPIO_SetBits(GPIOA, GPIO_Pin_5);
    GPIO_SetBits(GPIOA, GPIO_Pin_6);
    GPIO_SetBits(GPIOA, GPIO_Pin_7);
    GPIO_SetBits(GPIOC, GPIO_Pin_4);
}

void led1_off(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_5);
}
void led1_on(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}
void led1_toggle(void)
{
    GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
}

void led2_off(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_6);
}
void led2_on(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_6);
}
void led2_toggle(void)
{
    GPIO_ToggleBits(GPIOA, GPIO_Pin_6);
}

void led3_off(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_7);
}
void led3_on(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
}
void led3_toggle(void)
{
    GPIO_ToggleBits(GPIOA, GPIO_Pin_7);
}

void led4_off(void)
{
    GPIO_SetBits(GPIOC, GPIO_Pin_4);
}
void led4_on(void)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_4);
}
void led4_toggle(void)
{
    GPIO_ToggleBits(GPIOC, GPIO_Pin_4);
}
