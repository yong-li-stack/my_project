#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "unity.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_system.h"

#include "mpu_dmp_driver.h"


#define GPIO_MPU_INTR 4

extern float pitch, roll, yaw;   
static QueueHandle_t gpio_evt_queue = NULL; // 用于接收 GPIO 中断的 queue  

// 初始化所有需要使用的GPIO 
void gpio_init(void);
void gpio_task(void* arg);  
void gpio_intr_handle(void* arg);

void app_main(void)
{
    mpu_dmp_init();
    gpio_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));  
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_MPU_INTR, gpio_intr_handle, (void*) GPIO_MPU_INTR);

}


void gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_MPU_INTR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);

}

void gpio_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            gyro_data_ready_cb();
            dmp_get_data();
            printf("pitch:%f,roll:%f,yaw:%f\n",pitch, roll, yaw);
            // expression.frame_delay_ms = (uint16_t)(esp_random() % 10);
            // if (abs(pitch) > 30)
            // {
            //     expression.forehead_expr = arc;
            //     expression.eyes_expr = wink;
            // }
            // else  
            // {
            //     expression.forehead_expr = forehead_none;
            //     expression.eyes_expr = eyes_none;
            // }
            // oled_refresh_expression(expression);
        }
    }
}

void gpio_intr_handle(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}