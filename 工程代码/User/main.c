#include "stm32f10x.h"                  // Device header
#include <math.h>    // 提供 pow() 函数
#include <stdlib.h>  // 提供 abs() 函数 
#include "Delay.h"
#include "Motor.h"
#include "Servo.h"
#include "Serial.h"
#include "UltraSound.h"
#include "Track.h"

#define waitTime 300

uint16_t mode;
uint8_t running_statu;

#define SCAN_STEP     30    // 角度扫描步长
#define MIN_DISTANCE  30    // 触发避障的最小距离(cm)
#define SAFE_ANGLE    90    // 舵机回中角度

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    // 计算线性基准值
    int linear = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
    // 添加非线性补偿（二次曲线增强小角度时间）
    float compensation = 1.0 + pow((float)(abs(x - in_min)) / (in_max - in_min), 2);
    
    // 应用补偿并约束输出范围
    int result = (int)(linear * compensation);
    return (result < 500) ? 500 : (result > 1500) ? 1500 : result; // 强制500-1500ms范围
}

void ObstacleAvoidance() {
    float max_distance = 0;
    int optimal_angle = SAFE_ANGLE;
    
    // 多角度扫描（0°~180°，步长30°）
    for (int angle = 0; angle <= 180; angle += SCAN_STEP) {
        Servo_SetAngle(angle);
        Delay_ms(300);                  // 等待舵机稳定
        float current_dist = UltraSound_Measure();
        
        // 记录最大距离及对应角度
        if (current_dist > max_distance) {
            max_distance = current_dist;
            optimal_angle = angle;
        }
    }
    
    Servo_SetAngle(SAFE_ANGLE);         // 扫描完成回中
    Delay_ms(300);

    // 执行避障决策
    if (max_distance < MIN_DISTANCE) {
        // 所有方向均受阻
        MoveBackward();
        Delay_ms(800);
        Self_Right();                   // 右转180°调头（需实现）
        Delay_ms(500);
    } else {
        // 计算转向策略
        if (optimal_angle < SAFE_ANGLE) {
            Turn_Right();                // 最优方向在右侧
            Delay_ms(map(optimal_angle, 0, SAFE_ANGLE, 800, 300));  // 动态转向时长
        } else if (optimal_angle > SAFE_ANGLE) {
            Turn_Left();               // 最优方向在左侧
            Delay_ms(map(optimal_angle, SAFE_ANGLE, 180, 300, 800));
        } else {
            MoveForward();              // 最优方向正前方
        }
    }
}



int main(void)
{
	Motor_Init();
    Servo_Init();
    Serial_Init();
    UltraSound_Init();
    Track_Init();
    //0x10--手动模式   0x20--自动避障模式 0x40--循迹模式
    mode=0x10;
    /*  运行状态
        0x30--停止
        0x31--前进
        0x32--后退
        0x33--右转
        0x34--左转
        0x35--原地右转
        0x36--原地左转
    */
    running_statu=0x30;
    Servo_SetAngle(90);
    
	while (1)
	{
        
        //自动模式
        if(mode==0x20){
            float distance = UltraSound_Measure();
        if (distance < MIN_DISTANCE) {
            Car_Stop();
            ObstacleAvoidance();
        } else {
            MoveForward();
        }
        Delay_ms(100);

        }
        else if(mode==0x40){
            //循迹 检测到黑线输出高电平
            uint8_t rt = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);  // 最右
            uint8_t rm = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);  // 右中
            uint8_t lm = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);  // 左中
            uint8_t lt = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);  // 最左

            // 调整判断顺序，优先处理转弯条件
            if ((rt == 1 || rm == 1) && lm == 0 && lt == 0) {
                // 右侧检测到黑线，左转
                Turn_Left();
                Delay_ms(waitTime);
            }
            else if ((lm == 1 || lt == 1) && rt == 0 && rm == 0) {
                // 左侧检测到黑线，右转
                Turn_Right();
                Delay_ms(waitTime);
            }
            else if ((rt == 0 && lt == 0) && (rm == 1 || lm == 1)) {
                // 中间传感器触发且两侧未触发，继续直行（可能需要调整）
                MoveForward();
                Delay_ms(waitTime);
            }
            else if (rt == 0 && rm == 0 && lm == 0 && lt == 0) {
                // 所有传感器未检测到，保持直行或停止
                MoveForward(); // 或根据需求处理
                Delay_ms(waitTime);
            }
            else if (rt == 1 && rm == 1 && lm == 1 && lt == 1) {
                // 十字路口，停止
                Car_Stop();
                Delay_ms(waitTime);
            }
            
        }
        else{
        /*  运行状态
            0x30--停止
            0x31--前进
            0x32--后退
            0x33--右转
            0x34--左转
            0x35--原地右转
            0x36--原地左转
        */
            if(running_statu==0x31){
                MoveForward();
                Delay_ms(waitTime);
            }
            else if(running_statu==0x32){
                MoveBackward();
                Delay_ms(waitTime);
            }
            else if(running_statu==0x33){
                Turn_Right();
                Delay_ms(waitTime);
            }
            else if(running_statu==0x34){
                Turn_Left();
                Delay_ms(waitTime);
            }
            else if(running_statu==0x35){
                Self_Right();
                Delay_ms(waitTime);
            }
            else if(running_statu==0x36){
                Self_Left();
                Delay_ms(waitTime);
            }
            else{
                Car_Stop();
            }
            
            
        }      
	}
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
        uint16_t Data;
		Data=USART_ReceiveData(USART1);
        if(Data==0x10) mode=0x10;
        else if(Data==0x20) mode=0x20;
        else if(Data==0x30) running_statu=0x30;
        else if(Data==0x31) running_statu=0x31;
        else if(Data==0x32) running_statu=0x32;
        else if(Data==0x33) running_statu=0x33;
        else if(Data==0x34) running_statu=0x34;
        else if(Data==0x35) running_statu=0x35;
        else if(Data==0x36) running_statu=0x36;
        else if(Data==0x40) mode=0x40;
        else if(Data==0x50){
            Servo_SetAngle(0);
            Delay_ms(500);
            Servo_SetAngle(90);
            Delay_ms(500);
            Servo_SetAngle(180);
            Delay_ms(500);
        }
	}
}

