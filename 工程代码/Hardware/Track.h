#ifndef __TRACK_H
#define __TRACK_H
// 引脚定义（假设从右到左排列：Pin5=最右侧，Pin8=最左侧）
#define RT_SENSOR    GPIO_Pin_5  // 最右侧传感器
#define RM_SENSOR    GPIO_Pin_6  // 右侧中间
#define LM_SENSOR    GPIO_Pin_7  // 左侧中间 
#define LT_SENSOR    GPIO_Pin_8  // 最左侧传感器
void Track_Init(void);

#endif 
