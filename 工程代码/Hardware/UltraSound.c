#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/*超声波测距*/

#define Trig GPIO_Pin_12
#define Echo GPIO_Pin_13

void UltraSound_Init(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = Trig;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = Echo;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
    
    TIM_InternalClockConfig(TIM4);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 60000 - 1;                 //计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;               //预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元    
}
 
float UltraSound_Measure(void){
    float cnt,distance;
    GPIO_SetBits(GPIOB,Trig);
    Delay_us(20);
    GPIO_ResetBits(GPIOB,Trig);
    while(GPIO_ReadInputDataBit(GPIOB,Echo)==RESET);
    TIM_Cmd(TIM4,ENABLE);
    while(GPIO_ReadInputDataBit(GPIOB,Echo)==SET);
    TIM_Cmd(TIM4,DISABLE);
    cnt=TIM_GetCounter(TIM4);
     distance=(cnt*1.0/10*0.34)/2;
    TIM4->CNT=0;
    Delay_ms(100);
    return distance;
}

