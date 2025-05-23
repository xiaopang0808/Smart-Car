#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Motor.h"
#include "Servo.h"
#include "Serial.h"
#include "UltraSound.h"
#include "Track.h"

#define waitTime 300
#define Delay_time 200

uint16_t mode;
uint8_t running_statu;

int main(void)
{
	Motor_Init();
    Servo_Init();
    Serial_Init();
    UltraSound_Init();
    Track_Init();
//    0x10--手动模式  
//    0x20--自动避障模式
//    0x30--循迹模式
    mode=0x10;
    /*  运行状态
        0x11--前进
        0x12--后退
        0x13--右转
        0x14--左转
        0x15--原地右转
        0x16--原地左转
        0x17--停止
    */
    running_statu=0x17;
    Servo_SetAngle(90);      
	while (1)
	{
        //超声波避障模式
        if(mode==0x20){
            Set_Car_Speed(100);
            float distance = UltraSound_Measure();
            if (distance < MIN_DISTANCE){
                Car_Stop();
                ObstacleAvoidance();
            } else {
                MoveForward();
            }
            Delay_ms(100);
        }
        //循迹模式
        else if(mode==0x30){
            Set_Car_Speed(100);
            //循迹 检测到黑线输出高电平
            uint8_t rt=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5);
            uint8_t rm=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
            uint8_t lm=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
            uint8_t lt=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8);
            if(lt==0 && rt==1){
                Car_Stop();
                while(rm==0 && lm==0){
                    rm=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
                    lm=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
                    Self_Right(); 
                }
            }
            else if(rt==0 && lt==1){
                Car_Stop();
                
                while(rm==0 && lm==0){
                    rm=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
                    lm=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
                    Self_Left();
                }
            }
            else{
                if(rm==0 && lm==1){
                    Car_Stop();
                    Turn_Left();
                }
                else if(lm==0 && rm==1){
                    Car_Stop();
                    Turn_Right(); 
                }
                else{
                    MoveForward();
                } 
            }        
        }
        //手动模式
        else{
        /*  运行状态
        0x11--前进
        0x12--后退
        0x13--右转
        0x14--左转
        0x15--原地右转
        0x16--原地左转
        0x17--停止
        */
            Set_Car_Speed(100);
            if(running_statu==0x11){
                MoveForward();
            }
            else if(running_statu==0x12){
                MoveBackward();
            }
            else if(running_statu==0x13){
                Turn_Right();
            }
            else if(running_statu==0x14){
                Turn_Left();
            }
            else if(running_statu==0x15){
                Self_Right();
            }
            else if(running_statu==0x16){
                Self_Left();
            }
            else{
                Car_Stop();
            }
            Delay_ms(Delay_time);
            
        }      
	}
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
        uint16_t Data;
		Data=USART_ReceiveData(USART1);
        if(Data==0x10) mode=0x10;                   //手动模式
        /*  运行状态
        0x11--前进
        0x12--后退
        0x13--右转
        0x14--左转
        0x15--原地右转
        0x16--原地左转
        0x17--停止
        */
        else if(Data==0x11) running_statu=0x11;     
        else if(Data==0x12) running_statu=0x12;
        else if(Data==0x13) running_statu=0x13;
        else if(Data==0x14) running_statu=0x14;
        else if(Data==0x15) running_statu=0x15;
        else if(Data==0x16) running_statu=0x16;
        else if(Data==0x17) running_statu=0x17;
        else if(Data==0x20) mode=0x20;              //超声波避障
        else if(Data==0x30) mode=0x30;              //循迹模式
	}
}

