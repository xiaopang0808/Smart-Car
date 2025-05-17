#include "stm32f10x.h"                  // Device header
#include "PWM.h"
/*电机驱动*/
#define left1 GPIO_Pin_4
#define left2 GPIO_Pin_5
#define right1 GPIO_Pin_6
#define right2 GPIO_Pin_7

/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = left1 | left2 | right1 | right2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						//将PA4和PA5引脚初始化为推挽输出	
	
	PWM_Init();													//初始化直流电机的底层PWM
}


void Motor_SetLeftSpeed(int8_t Speed)
{
	if (Speed > 0)							
	{
		GPIO_SetBits(GPIOA, left1);	
		GPIO_ResetBits(GPIOA, left2);	
		PWM_SetCompare3(Speed);				
	}
    else if(Speed==0){
        GPIO_SetBits(GPIOA, left1);	
		GPIO_SetBits(GPIOA, left2);	
		PWM_SetCompare3(Speed);	
    }
	else									
	{
		GPIO_ResetBits(GPIOA, left1);	
		GPIO_SetBits(GPIOA, left2);	
		PWM_SetCompare3(-Speed);			
	}
}

void Motor_SetRightSpeed(int8_t Speed)
{
	if (Speed > 0)							
	{
		GPIO_SetBits(GPIOA, right1);
		GPIO_ResetBits(GPIOA, right2);	
		PWM_SetCompare3(Speed);	
    }        
    else if(Speed==0){
        GPIO_SetBits(GPIOA, right1);	
		GPIO_SetBits(GPIOA, right2);	
		PWM_SetCompare3(Speed);	
    }
	else									
	{
		GPIO_ResetBits(GPIOA, right1);	
		GPIO_SetBits(GPIOA, right2);	
		PWM_SetCompare3(-Speed);			
	}
}

void MoveForward(void){
    Motor_SetLeftSpeed(100);
    Motor_SetRightSpeed(100);
}

void MoveBackward(void){
    Motor_SetLeftSpeed(-100);
    Motor_SetRightSpeed(-100);
}

void Turn_Left(void){
    Motor_SetLeftSpeed(100);   
} 

void Turn_Right(void){
    Motor_SetRightSpeed(100); 
}

void Car_Stop(void){
    Motor_SetLeftSpeed(0);
    Motor_SetRightSpeed(0);   
}

void Self_Right(void){
    Motor_SetLeftSpeed(-100);
    Motor_SetRightSpeed(100); 
}

void Self_Left(void){
    Motor_SetLeftSpeed(100);
    Motor_SetRightSpeed(-100); 
}
