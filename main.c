/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"
#include "main.h"

#define PWM_PERIOD_MAX   8000

#define MOTOR_LEFT       0x01   // 左轮电机ID
#define MOTOR_RIGHT      0x02   // 右轮电机ID

#define MOTOR_STOP       0x00   // 停止
#define MOTOR_FORWARD    0x01   // 正转 (前进)
#define MOTOR_REVERSE    0x02   // 反转 (后退)

void TB6612_SetMotorDirection(uint8_t motor_id, uint8_t direction);
void TB6612_SetMotorSpeed(uint8_t motor_id, uint32_t speed_percent);
uint32_t Set_Circles();
void Set_Path(uint32_t count);
void MakePreciseTurn() ;
bool FollowLineWithCorrection();

int main(void)
    {
    SYSCFG_DL_init();
    SysTick_Init();
    // MPU6050_Init();
    OLED_Init();
    // Ultrasonic_Init();
    // BNO08X_Init();
    // WIT_Init();
    // VL53L0X_Init();
    // LSM6DSV16X_Init();
    // IMU660RB_Init();

    /* Don't remove this! */
    Interrupt_Init();
    DL_GPIO_setPins(GPIO_6612_STBY_PORT, GPIO_6612_STBY_PIN);
    DL_TimerA_startCounter(PWM_WHEEL_INST);
    uint32_t count=0;
    count=Set_Circles();
    for (int i = 0; i < count * 4; i++) {
        FollowLineWithCorrection();
    }

    TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_STOP);
    TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_STOP);
    while (1) 
    {

    }
}
    typedef enum {
    LINE_FOLLOWING,
    PRE_CORNER,
    CORNER_CONFIRM
    } CornerState;
    static CornerState corner_state = LINE_FOLLOWING;
    static int confirm_count = 0;
bool FollowLineWithCorrection() {

    // int stable_corner_count = 0;
    // const int stable_threshold = 22;  // 连续 5 次认为是真正路口
    while (1) {
        uint8_t s1_val = (DL_GPIO_readPins(GPIO_SENSOR_PORT, GPIO_SENSOR_SENSOR_LEFTMOST_PIN0_PIN) ? 1 : 0);
        uint8_t s2_val = (DL_GPIO_readPins(GPIO_SENSOR_PORT, GPIO_SENSOR_SENSOR_LEFT_PIN0_PIN) ? 1 : 0);
        uint8_t s3_val = (DL_GPIO_readPins(GPIO_SENSOR_PORT, GPIO_SENSOR_SENSOR_CENTER_PIN0_PIN) ? 1 : 0);
        uint8_t s4_val = (DL_GPIO_readPins(GPIO_SENSOR_PORT, GPIO_SENSOR_SENSOR_RIGHT_PIN0_PIN) ? 1 : 0);
        uint8_t s5_val = (DL_GPIO_readPins(GPIO_SENSOR_PORT, GPIO_SENSOR_SENSOR_RIGHTMOST_PIN0_PIN) ? 1 : 0);

        uint8_t sensor_state = (~s1_val & 0x01) << 4 |
                               (~s2_val & 0x01) << 3 |
                               (~s3_val & 0x01) << 2 |
                               (~s4_val & 0x01) << 1 |
                               (~s5_val & 0x01) << 0;

    switch (corner_state) {
        case LINE_FOLLOWING:
            if (sensor_state == 0b11100 || sensor_state == 0b11000 || sensor_state == 0b11110) {
                corner_state = PRE_CORNER;
            }
            break;

        case PRE_CORNER:
            if (sensor_state == 0b00000 || sensor_state == 0b10001 || sensor_state == 0b11111) {
                confirm_count++;
                if (confirm_count >= 20) {
                    corner_state = LINE_FOLLOWING;
                    confirm_count = 0;
                    return true;  // 确认到达拐角
                }
            } else if (sensor_state == 0b00100 || sensor_state == 0b00010 || sensor_state == 0b01000) {
                corner_state = LINE_FOLLOWING;
                confirm_count = 0; // 假报警
            }
            break;
    }

        switch (sensor_state) {
            case 0b00100:
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_FORWARD);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                break;
            case 0b00001:
            case 0b00011:

                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_FORWARD);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_REVERSE);
                break;
            case 0b10000:
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_STOP);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                break;
  
            case 0b11100:
            case 0b11110:
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_FORWARD);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                delay_ms(91);
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_REVERSE);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                delay_ms(280);
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_STOP);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_STOP);
                delay_ms(700); // 确保稳定停下

                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_FORWARD);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                delay_ms(250);
                break;
            case 0b00010:
            case 0b00110:          
            case 0b11000:
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_FORWARD);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_STOP);
                break;
            case 0b01000:
            case 0b01100:
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_STOP);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                break;
            case 0b00000:
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_FORWARD);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                break;
            default:
                TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_STOP);
                TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_STOP);
                break;
        }

        delay_ms(5); // 加入延迟以防刷新太快
    }
}


void MakePreciseTurn() {
    // 原地左转（逆时针）    
    TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_STOP);
    TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_STOP);
    delay_ms(100);
    TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_REVERSE);
    TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
    delay_ms(50);  // 视角度调试

}

uint32_t Set_Circles(){
    uint32_t cnt=0;
    while (DL_GPIO_readPins(GPIO_BUTTON_PIN_2_PORT, GPIO_BUTTON_PIN_2_PIN)) {
        OLED_ShowChinese(0, 0, 6, 16);  // "设置"
        OLED_ShowChinese(16, 0, 7, 16); // "圈"
        OLED_ShowChinese(32, 0, 8, 16); // "数"
        OLED_ShowNum(48, 0, cnt, 2, 16);
        
        // 按键1按下（低电平有效）
        if (DL_GPIO_readPins(GPIO_BUTTON_PIN_1_PORT, GPIO_BUTTON_PIN_1_PIN) == 0) {
            delay_ms(20); // 消抖
            while (DL_GPIO_readPins(GPIO_BUTTON_PIN_1_PORT, GPIO_BUTTON_PIN_1_PIN) == 0);
            delay_ms(20);
            DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_0_PIN);
            cnt++;
        }
        if(DL_GPIO_readPins(GPIO_SENSOR_PORT, GPIO_SENSOR_SENSOR_LEFTMOST_PIN0_PIN) != 0){
            delay_ms(20);
            DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_0_PIN);
            OLED_ShowString(0,2,(uint8_t *)"OK",16);
            break;
        }
    }
    return cnt;
}
void Set_Path(uint32_t count){
    for(uint8_t i=0;i<count*4;i++){
        TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_FORWARD);
        TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
        delay_ms(1500);
        TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_STOP);
        TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_STOP);
        delay_ms(100);
        TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_REVERSE);
        TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_FORWARD);
        delay_ms(600);
        TB6612_SetMotorDirection(MOTOR_LEFT, MOTOR_STOP);
        TB6612_SetMotorDirection(MOTOR_RIGHT, MOTOR_STOP);
        delay_ms(100);
    }
}
void TB6612_SetMotorDirection(uint8_t motor_id, uint8_t direction)
{
    // 如果是左电机 (MOTOR_LEFT)
    if (motor_id == MOTOR_LEFT)
    {
        if (direction == MOTOR_STOP) // 停止
        {
            DL_GPIO_clearPins(GPIO_6612_AIN_1_PORT, GPIO_6612_AIN_1_PIN); // AIN1 设为低
            DL_GPIO_clearPins(GPIO_6612_AIN_2_PORT, GPIO_6612_AIN_2_PIN); // AIN2 设为低
        }
        else if (direction == MOTOR_FORWARD) // 正转 (根据你给出的测试代码，AIN1 高 AIN2 低为正转)
        {
            DL_GPIO_setPins(GPIO_6612_AIN_1_PORT, GPIO_6612_AIN_1_PIN);   // AIN1 设为高
            DL_GPIO_clearPins(GPIO_6612_AIN_2_PORT, GPIO_6612_AIN_2_PIN); // AIN2 设为低
        }
        else if (direction == MOTOR_REVERSE) // 反转 (根据你给出的测试代码，AIN1 低 AIN2 高为反转)
        {
            DL_GPIO_clearPins(GPIO_6612_AIN_1_PORT, GPIO_6612_AIN_1_PIN); // AIN1 设为低
            DL_GPIO_setPins(GPIO_6612_AIN_2_PORT, GPIO_6612_AIN_2_PIN);   // AIN2 设为高
        }
    }
    // 如果是右电机 (MOTOR_RIGHT)
    else if (motor_id == MOTOR_RIGHT)
    {
        if (direction == MOTOR_STOP) // 停止
        {
            DL_GPIO_clearPins(GPIO_6612_BIN_1_PORT, GPIO_6612_BIN_1_PIN); // BIN1 设为低
            DL_GPIO_clearPins(GPIO_6612_BIN_2_PORT, GPIO_6612_BIN_2_PIN); // BIN2 设为低
        }
        else if (direction == MOTOR_FORWARD) // 正转
        {
            DL_GPIO_setPins(GPIO_6612_BIN_1_PORT, GPIO_6612_BIN_1_PIN);   // BIN1 设为高
            DL_GPIO_clearPins(GPIO_6612_BIN_2_PORT, GPIO_6612_BIN_2_PIN); // BIN2 设为低
        }
        else if (direction == MOTOR_REVERSE) // 反转
        {
            DL_GPIO_clearPins(GPIO_6612_BIN_1_PORT, GPIO_6612_BIN_1_PIN); // BIN1 设为低
            DL_GPIO_setPins(GPIO_6612_BIN_2_PORT, GPIO_6612_BIN_2_PIN);   // BIN2 设为高
        }
    }
}

void TB6612_SetMotorSpeed(uint8_t motor_id, uint32_t speed_percent)
{
    uint32_t compare_value = (uint32_t)((PWM_PERIOD_MAX * speed_percent) / 100.0f);

    if (motor_id == MOTOR_LEFT)
    {

        DL_TimerA_setCaptureCompareValue(PWM_WHEEL_INST, compare_value,GPIO_PWM_WHEEL_C0_IDX);
    }
    else if (motor_id == MOTOR_RIGHT)
    {
        DL_TimerA_setCaptureCompareValue(PWM_WHEEL_INST, compare_value,GPIO_PWM_WHEEL_C1_IDX );
    }
}