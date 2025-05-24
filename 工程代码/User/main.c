#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Motor.h"
#include "Servo.h"
#include "Serial.h"
#include "UltraSound.h"
#include "Track.h"

#define waitTime 300
#define Delay_time 100

uint16_t mode;
uint8_t running_statu;

// 全局状态标记（需在外部定义）
enum TrackState { NORMAL, SHARP_TURN }; 
enum TrackState track_state = NORMAL;

void TrackingProcess(void)
{
    uint8_t rt = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
    uint8_t rm = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
    uint8_t lm = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
    uint8_t lt = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);

    /*--------------- 常规循迹模式 ---------------*/
    if (track_state == NORMAL) 
    {
        // 优先级1：外侧传感器触发（可能进入直角弯）
        if (rt || lt) 
        {
            track_state = SHARP_TURN;  // 切换到大弯模式
            return;  // 本周期不处理，等待下个周期进入SHARP_TURN
        }

        // 优先级2：中间传感器微调
        if (rm) {
            Turn_Right();  // 向右微调（差速转向）
        } 
        else if (lm) {
            Turn_Left();   // 向左微调
        } 
        else {
            MoveForward();  // 无检测时直行
        }
    }

    /*--------------- 大角度弯道模式 ---------------*/
    else if (track_state == SHARP_TURN) 
    {
        // 持续读取传感器（动态响应）
        rt = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
        lt = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);

        // Step1: 判断弯道方向
        if (rt) 
        {
            // 右直角弯处理：持续右转直到中间传感器捕获
            while (1) 
            {
                Self_Right();  // 大幅右转（如右轮反转）
                Delay_ms(50); // 保持转向动作
                
                // 退出条件：中间传感器重新检测到黑线
                if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)) {
                    track_state = NORMAL;
                    break;
                }
            }
        } 
        else if (lt) 
        {
            // 左直角弯处理：持续左转直到中间传感器捕获
            while (1) 
            {
                Self_Left();   // 大幅左转
                Delay_ms(50);
                
                if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)) {
                    track_state = NORMAL;
                    break;
                }
            }
        }
        
        // 意外退出保护
        track_state = NORMAL;
    }
}

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
            
            TrackingProcess();
            
            //循迹 检测到黑线输出高电平
//            uint8_t rt=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5);
//            uint8_t rm=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
//            uint8_t lm=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
//            uint8_t lt=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8);
//                
//            
//            if(lt==lm && lm==rm && rm==rt){
//                if(lt==1){
//                    Car_Stop();
//                }
//                else{
//                    MoveForward();
//                }
//            }
            


            
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
            else if(running_statu==0x17){
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

void USART3_IRQHandler(void){
    if (USART_GetITStatus(USART3, USART_IT_RXNE) ==SET)		//判断是否是USART3的接收事件触发的中断
	{
        uint16_t Data;
		Data=USART_ReceiveData(USART3);
        if(Data==0x10) mode=0x10;                   //手动模式
        /*  运行状态
        0x11--齐步走
        0x12--后退
        0x13--右转
        0x14--左转
        0x15--向右转
        0x16--向左转
        0x17--立正
        */
        else if(Data==0x11){
            MoveForward();
            Delay_ms(500);
            Car_Stop();
        }            
        else if(Data==0x12){
            MoveBackward();
            Delay_ms(500);
            Car_Stop();
        }      
        else if(Data==0x13){
            Turn_Right();
            Delay_ms(300);
            Car_Stop();
        }      
        else if(Data==0x14){
            Turn_Left();
            Delay_ms(300);
            Car_Stop();
        }      
        else if(Data==0x15){
            Self_Right();
            Delay_ms(500);
            Car_Stop();
        }      
        else if(Data==0x16){
            Self_Left();
            Delay_ms(500);
            Car_Stop();
        }      
        else if(Data==0x17){
            Car_Stop();
        }      
        else if(Data==0x20) mode=0x20;              //超声波避障
        else if(Data==0x30) mode=0x30;              //循迹模式
	}
}




