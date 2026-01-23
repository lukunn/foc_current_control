/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_simplefoc.h"


BLDCMotor motor = BLDCMotor(22/2);
BLDCDriver6PWM driver6 = BLDCDriver6PWM(12,7, 11,6, 9,8);
AS5600 as5600 = AS5600(I2C_NUM_0, GPIO_NUM_2, GPIO_NUM_10);

float target_value = 0.0f;
Commander command = Commander(Serial);
void doTarget(char *cmd)
{
    command.scalar(&target_value, cmd);
}

extern "C" void app_main(void)
{
    SimpleFOCDebug::enable();                                        /*!< Enable debug */
    Serial.begin(115200);

    as5600.init();                                                   /*!< Enable as5600 */
    motor.linkSensor(&as5600);
    driver6.voltage_power_supply = 15;
    driver6.voltage_limit = 14;
    driver6.init();
    
    motor.linkDriver(&driver6);
    motor.controller = MotionControlType::velocity;                  /*!< Set position control mode */
    /*!< Set velocity pid */
    motor.PID_velocity.P = 0.9f;
    motor.PID_velocity.I = 2.2f;
    motor.voltage_limit = 14;
    motor.voltage_sensor_align = 2;
    motor.LPF_velocity.Tf = 0.05;
    motor.velocity_limit = 200;

    motor.useMonitoring(Serial);
    motor.init();                                                    /*!< Initialize motor */
    motor.initFOC();                                                 /*!<  Align sensor and start FOC */
    command.add('T', doTarget, const_cast<char *>("target angle"));  /*!< Add serial command */
    // "T10 set 10rad/s"
    // "T0  set  0rad/s"   
    while (1) {
        motor.loopFOC();
        motor.move(target_value);
        command.run();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
