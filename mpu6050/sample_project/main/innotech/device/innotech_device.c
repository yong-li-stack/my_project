/******************** (C) COPYRIGHT 2021 INNOTECH **************************
* COMPANY:			INNOTECH
* DATE:				2021/08
* AUTHOR:			qiang.zhang
* IC:				ESP32S3
* DESCRIPTION:	    Device Service Handle.
*____________________________________________________________________________
* REVISION  Date		    User            Description
* 1.0		2021/08/29	    qiang.zhang		First release
*
*____________________________________________________________________________

*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void innotech_device_service_start(void)
{
    
}

void innotech_device_service_handle(void *args)
{
    while(1)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
       /*static int tick = 0;
        if(++tick >= 50)
        {
            tick = 0;
            printf("heap size: %ld\r\n", innotech_get_heap_size());
        }*/
    }    
}

void innotech_device_service_init(void)
{    
    if(xTaskCreate(&innotech_device_service_handle, "device_service_handle", 5120, NULL, 23, NULL) != pdTRUE)
    {
        //ESP_LOGI(TAG, "create device process task failed!!!");
    }
}

void innotech_device_init(void)
{
	innotech_device_service_init();
    innotech_device_service_start();
}