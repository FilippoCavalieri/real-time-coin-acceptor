#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "Fotoresistore.h"
#include "Estensimetro.h"
#include "Servo.h"
#include "DimensionSensor.h"

#define CHANNEL_TIMEOUT 6000

Fotoresistore fotoresistore(26);
Estensimetro estensimetro(27);
Servo servo_1(19); 
Servo servo_2(21); 
DimensionSensor dimensionSensor(10, 12);

SemaphoreHandle_t strain_gauge_synch_sem = xSemaphoreCreateBinary();
SemaphoreHandle_t dimension_sensor_synch_sem = xSemaphoreCreateBinary();
SemaphoreHandle_t first_servo_synch_sem = xSemaphoreCreateBinary();



bool stopStrainGauge = false;
SemaphoreHandle_t stopStrainGaugeMutex = xSemaphoreCreateMutex();

void vPhotoresistorRead(void * params){
    int already_signaled = 0;
    for(;;){
        uint16_t result = fotoresistore.getLight();
        //printf("PHOTORESISTOR: Read value: hex: 0x%03x, dec: %d\n", result, result);
        if(result < 1000 && !already_signaled){
            already_signaled = 1;
            xSemaphoreGive(strain_gauge_synch_sem);
            xSemaphoreGive(first_servo_synch_sem);
            xSemaphoreGive(dimension_sensor_synch_sem);
        }
        else if(already_signaled && result > 1000){
            already_signaled = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }
}


void vStrainGaugeRead(void * params){
    int counter;
    for(;;){
        xSemaphoreTake(strain_gauge_synch_sem, portMAX_DELAY);
        printf("STRAIN GAUGE: start\n");
        for(;;){
            xSemaphoreTake(stopStrainGaugeMutex, portMAX_DELAY);
            if (stopStrainGauge) {
                stopStrainGauge = false;
                xSemaphoreGive(stopStrainGaugeMutex);
                break;
            }
            xSemaphoreGive(stopStrainGaugeMutex);
            uint16_t result = estensimetro.getWeight();
            printf("w%d\n", result);
            vTaskDelay(pdMS_TO_TICKS(5UL));
        }
        printf("STRAIN GAUGE: stop\n");
    }
}

void vDimensionSensorRead(void * params){
    for(;;){
        xSemaphoreTake(dimension_sensor_synch_sem, portMAX_DELAY);
        TickType_t startTimeChannel = xTaskGetTickCount();
        TickType_t durationChannel = 0;
        bool result = false;
        printf("DIMENSION SENSOR: wait overlap\n");
        while (!result && durationChannel < CHANNEL_TIMEOUT) {
            result = dimensionSensor.getOverlap();
            //vTaskDelay(pdMS_TO_TICKS(1UL));
            durationChannel = xTaskGetTickCount() - startTimeChannel;
        }
        TickType_t startTimeOverlap = xTaskGetTickCount();
        //printf("DIMENSION SENSOR: overlap\n");
        xSemaphoreTake(stopStrainGaugeMutex, portMAX_DELAY);
        stopStrainGauge = true;
        xSemaphoreGive(stopStrainGaugeMutex);
        //printf("DIMENSION SENSOR: wait no overlap\n");
        while (result) {
            result = dimensionSensor.getOverlap();
            //vTaskDelay(pdMS_TO_TICKS(1UL));
        }
        //printf("DIMENSION SENSOR: no overlap\n");
        TickType_t durationOverlap = xTaskGetTickCount() - startTimeOverlap;
        if(durationOverlap > 135){
            servo_2.goDegree(4); // 2 euro
        }
        else if(durationOverlap > 95){ //20 c
            servo_2.goDegree(32);
        }else if(durationOverlap > 20){
             servo_2.goDegree(60); // 1 c
        }
        else{
            servo_2.goDegree(88);
        }
        if(durationChannel >= CHANNEL_TIMEOUT ){
            printf("t0\n");
        }else if( durationOverlap <= 3 ){
            printf("t-100\n");
        }
        else{
            printf("t%d\n", durationOverlap);
        }
        printf("c%d\n", durationChannel);
    }
}


void vFirstServoAction(void * params){
    for(;;){
        servo_1.goDegree(90);
        xSemaphoreTake(first_servo_synch_sem, portMAX_DELAY);
        servo_1.goDegree(135);
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }  
}

void initialize_board(){
    stdio_init_all();
}

int main(){
    initialize_board();
    servo_2.goDegree(4);
    xTaskCreate(vPhotoresistorRead, "Entry section photoresistor's read", 1024, NULL, 10, NULL);
    xTaskCreate(vStrainGaugeRead, "Strain gauge's read", 1024, NULL, 8, NULL);
    xTaskCreate(vDimensionSensorRead, "DimensionSensor's read", 1024, NULL, 7, NULL);
    xTaskCreate(vFirstServoAction, "Entry section blade", 1024, NULL, 9, NULL);
    vTaskStartScheduler();
    return 0;
}