#include "stm32f10x.h"                  // Device header
#include "TIM4.h"
#include "PID.h"
#include "math.h"
#include "Motor.h"
#include "BUZZER.h"
#include "delay.h"
#include "OLED.h"



#define MOVE_COUNT 25

// 状态定义
typedef enum 
{
    TRIANGLE_STATE_MOVE1,   // 第一条边
    TRIANGLE_STATE_TURN1,   // 第一次转向
    TRIANGLE_STATE_MOVE2,   // 第二条边
    TRIANGLE_STATE_TURN2,   // 第二次转向
    TRIANGLE_STATE_MOVE3,   // 第三条边
    TRIANGLE_STATE_STOP     // 停止
} TriangleState;

typedef enum 
{
    Rectangle_STATE_MOVE1,   // 第一条边
    Rectangle_STATE_TURN1,   // 第一次转向
    Rectangle_STATE_MOVE2,   // 第二条边
    Rectangle_STATE_TURN2,   // 第二次转向
    Rectangle_STATE_MOVE3,   // 第三条边
    Rectangle_STATE_TURN3,
	Rectangle_STATE_MOVE4,
	Rectangle_STATE_STOP     // 停止
} Rectangle;


uint8_t Mode_Car = 1;		//默认模式1（直线模式），模式2（三角形模式），模式3（平行四边形模式）
Rectangle rec_state = Rectangle_STATE_MOVE1;
TriangleState tri_state = TRIANGLE_STATE_MOVE1;
float target_angle = 0.0f;
uint16_t move_count = MOVE_COUNT;
uint8_t PID_Turn_Flag = 1;//第一次传参的标志位,用于更新转弯的目标角和重置输出值

float angle_diff(float target, float current) 
{
	float diff = target - current;
	
	 if (diff >= 180.0f) 
	{
        diff -= 360.0f;
    } 
	else if (diff < -180.0f) 
	{
        diff += 360.0f;
    }	

    return diff;
}



void Mode_Triangle(float* yaw, int16_t* State) 
{
	static uint16_t PID_Flag = 0;/*判断是否是首次进入循环，以此来判断是否
								需要在PID中进行目标角度的更新和输出值的重置*/

	switch (tri_state) 
	{
		
        case TRIANGLE_STATE_MOVE1:
            if (move_count > 0)
			{
				OLED_ShowString(2, 1, "MOVE1");
                if (IT_Flag == 1) 
				{
                    IT_Flag = 0;
					// 判断是否是第一次调用（move_count 初始为 MOVE_COUNT）
					PID_Flag = (move_count == MOVE_COUNT) ? 1 : 0;
                    move_count--;
                    PID(yaw, State, PID_Flag, 1, PID_Flag);
                }
            }
			else 
			{
				Motor_Stop();
				
                target_angle = *yaw + 120.0f; //要左转
                tri_state = TRIANGLE_STATE_TURN1;
                move_count = MOVE_COUNT; // 重置计数器供后续使用

				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();
            }
            break;

        case TRIANGLE_STATE_TURN1:
            if (fabs(angle_diff(target_angle, *yaw)) > 8.0f) 
			{	
				OLED_ShowString(2, 1, "TURN1");
				OLED_ShowString(3, 1, "Diffr:");
				OLED_ShowSignedNum(3, 7, angle_diff(target_angle, *yaw), 5);
				
				if (PID_Turn_Flag == 1)
				{
					PID(&target_angle, State, 1, 2, 1);//转弯时直接传入目标角度
					PID_Turn_Flag = 0;
				}
				PID(yaw, State, 0, 2, 0); //不更新目标角，转弯PID，不重置输出值               
            } 
			else
			{
				Motor_Stop();
				
				tri_state = TRIANGLE_STATE_MOVE2;
				move_count = MOVE_COUNT;
				PID_Turn_Flag = 1;
				
				OLED_Clear();
				
            }
            break;

        case TRIANGLE_STATE_MOVE2:
            if (move_count > 0) 
			{
				OLED_ShowString(2, 1, "MOVE2");
                if (IT_Flag == 1) 
				{
                    IT_Flag = 0;
					PID_Flag = (move_count == MOVE_COUNT) ? 1 : 0;
                    move_count--;
                    PID(yaw, State, PID_Flag, 1, PID_Flag);
                }
            } 
			else 
			{
				Motor_Stop();
				
                target_angle = *yaw + 60.0f;
                tri_state = TRIANGLE_STATE_TURN2;
                move_count = MOVE_COUNT;
				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();

            }
            break;

        case TRIANGLE_STATE_TURN2:
            if (fabs(angle_diff(target_angle, *yaw)) > 8.0f) 
			{
				OLED_ShowString(2, 1, "TURN2");
				OLED_ShowString(3, 1, "Diffr:");
				OLED_ShowSignedNum(3, 7, angle_diff(target_angle, *yaw), 5);
				if (PID_Turn_Flag == 1)
				{
					PID(&target_angle, State, 1, 2, 1);
					
					PID_Turn_Flag = 0;
				}
				PID(yaw, State, 0, 2, 0);

            } 
			else 
			{
				Motor_Stop();
				
				tri_state = TRIANGLE_STATE_MOVE3;
				move_count = MOVE_COUNT;
				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();

				
				OLED_Clear();
				
            }
            break;

        case TRIANGLE_STATE_MOVE3:
            if (move_count > 0) 
			{
				OLED_ShowString(2, 1, "MOVE3");
                if (IT_Flag == 1) 
				{
                    IT_Flag = 0;
					PID_Flag = (move_count == MOVE_COUNT) ? 1 : 0;
                    move_count--;
                    PID(yaw, State, PID_Flag, 1, PID_Flag);
                }
            } 
			else 
			{
				Motor_Stop();
				
                tri_state = TRIANGLE_STATE_STOP;
				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();

            }
            break;

        case TRIANGLE_STATE_STOP:
            Motor_Stop();
			OLED_ShowString(2, 1, "TURN1");
			OLED_ShowString(3, 1, "Diffr:");
			OLED_ShowSignedNum(3, 7, angle_diff(target_angle, *yaw), 5);

            break;

        default:
            tri_state = TRIANGLE_STATE_STOP;
            break;
	}

}

/*-------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------*/


void Mode_Rectangle(float* yaw, int16_t* State) 
{
	static uint16_t PID_Flag = 0;/*判断是否是首次进入循环，以此来判断是否
								需要在PID中进行目标角度的更新和输出值的重置*/

	
	switch (rec_state) 
	{
		
        case Rectangle_STATE_MOVE1:
            if (move_count > 0)
			{
				OLED_ShowString(2, 1, "MOVE1");
                if (IT_Flag == 1) 
				{
                    IT_Flag = 0;
					// 判断是否是第一次调用（move_count 初始为 MOVE_COUNT）
					PID_Flag = (move_count == MOVE_COUNT) ? 1 : 0;
                    move_count--;
                    PID(yaw, State, PID_Flag, 1, PID_Flag);
                }
            }
			else 
			{
				Motor_Stop();
				
                target_angle = *yaw + 120.0f; //要左转
                rec_state = Rectangle_STATE_TURN1;
                move_count = MOVE_COUNT; // 重置计数器供后续使用

				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();
            }
            break;

        case Rectangle_STATE_TURN1:
            if (fabs(angle_diff(target_angle, *yaw)) > 8.0f) 
			{	
				OLED_ShowString(2, 1, "TURN1");
				OLED_ShowString(3, 1, "Diffr:");
				OLED_ShowSignedNum(3, 7, angle_diff(target_angle, *yaw), 5);
				
				if (PID_Turn_Flag == 1)
				{
					PID(&target_angle, State, 1, 2, 1);//转弯时直接传入目标角度
					PID_Turn_Flag = 0;
				}
				PID(yaw, State, 0, 2, 0); //不更新目标角，转弯PID，不重置输出值
                
            } 
			else
			{
				Motor_Stop();
				
				rec_state = Rectangle_STATE_MOVE2;
				move_count = MOVE_COUNT;
				PID_Turn_Flag = 1;
				
				OLED_Clear();
				
            }
            break;

        case Rectangle_STATE_MOVE2:
            if (move_count > 0) 
			{
				OLED_ShowString(2, 1, "MOVE2");
                if (IT_Flag == 1) 
				{
                    IT_Flag = 0;
					PID_Flag = (move_count == MOVE_COUNT) ? 1 : 0;
                    move_count--;
                    PID(yaw, State, PID_Flag, 1, PID_Flag);
                }
            } 
			else 
			{
				Motor_Stop();
				
                target_angle = *yaw - 120.0f;
                rec_state = Rectangle_STATE_TURN2;
                move_count = MOVE_COUNT;
				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();

            }
            break;

        case Rectangle_STATE_TURN2:
            if (fabs(angle_diff(target_angle, *yaw)) > 8.0f) 
			{
				OLED_ShowString(2, 1, "TURN2");
				OLED_ShowString(3, 1, "Diffr:");
				OLED_ShowSignedNum(3, 7, angle_diff(target_angle, *yaw), 5);
				if (PID_Turn_Flag == 1)
				{
					PID(&target_angle, State, 1, 2, 1);
					PID_Turn_Flag = 0;
				}
				PID(yaw, State, 0, 2, 0);
            } 
			else 
			{
				Motor_Stop();
				
				rec_state = Rectangle_STATE_MOVE3;
				move_count = MOVE_COUNT;
				PID_Turn_Flag = 1;
				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();
				
            }
            break;

        case Rectangle_STATE_MOVE3:
            if (move_count > 0) 
			{
				OLED_ShowString(2, 1, "MOVE3");
                if (IT_Flag == 1) 
				{
                    IT_Flag = 0;
					PID_Flag = (move_count == MOVE_COUNT) ? 1 : 0;
                    move_count--;
                    PID(yaw, State, PID_Flag, 1, PID_Flag);
                }
            } 
			else 
			{
				Motor_Stop();
				
				target_angle = *yaw - 150.0f;
                rec_state = Rectangle_STATE_TURN3;
				move_count = MOVE_COUNT;

				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();

            }
            break;
			
			case Rectangle_STATE_TURN3:
            if (fabs(angle_diff(target_angle, *yaw)) > 15.0f) 
			{
				OLED_ShowString(2, 1, "TURN3");
				OLED_ShowString(3, 1, "Diffr:");
				OLED_ShowSignedNum(3, 7, angle_diff(target_angle, *yaw), 5);
				if (PID_Turn_Flag == 1)
				{
					PID(&target_angle, State, 1, 2, 1);
					
					PID_Turn_Flag = 0;
				}
				
				PID(yaw, State, 0, 2, 0);

            } 
			else 
			{
				Motor_Stop();
				
				rec_state = Rectangle_STATE_MOVE4;
				move_count = MOVE_COUNT;
				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();
            }
            break;

			case Rectangle_STATE_MOVE4:
            if (move_count > 0) 
			{
				OLED_ShowString(2, 1, "MOVE4");
                if (IT_Flag == 1) 
				{
                    IT_Flag = 0;
					PID_Flag = (move_count == MOVE_COUNT) ? 1 : 0;
                    move_count--;
                    PID(yaw, State, PID_Flag, 1, PID_Flag);
                }
            } 
			else 
			{
				Motor_Stop();
				
                rec_state = Rectangle_STATE_STOP;
				
				BUZZER_ON();
				delay_ms(100);
				BUZZER_OFF();
				
				OLED_Clear();

            }
            break;

        case Rectangle_STATE_STOP:
            Motor_Stop();
			OLED_ShowString(2, 1, "STOP");
			OLED_ShowString(3, 1, "Diffr:");
			OLED_ShowSignedNum(3, 7, angle_diff(target_angle, *yaw), 5);

            break;

        default:
            rec_state = Rectangle_STATE_STOP;
            break;
		
	}

}
