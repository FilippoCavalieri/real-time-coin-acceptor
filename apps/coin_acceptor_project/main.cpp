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
            xSemaphoreGive(dimension_sensor_synch_sem);
            xSemaphoreGive(first_servo_synch_sem);
            //printf("")
        }
        else if(already_signaled && result > 1000){
            already_signaled = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }
}


void vStrainGaugeRead(void * params){
    int counter;
    printf("string gauge before take\n");
    for(;;){
        xSemaphoreTake(strain_gauge_synch_sem, portMAX_DELAY);
        
        for(;;){
            printf("STRAIN GAUGE: start\n");
            xSemaphoreTake(stopStrainGaugeMutex, portMAX_DELAY);
            printf("b%d",stopStrainGauge);
            if (stopStrainGauge) {
                stopStrainGauge = false;
                xSemaphoreGive(stopStrainGaugeMutex);
                break;
            }
            xSemaphoreGive(stopStrainGaugeMutex);
            uint16_t result = estensimetro.getWeight();
            printf("w%d\n", result);
            printf("STRAIN GAUGE: stop\n");
            vTaskDelay(1);
        }
        
        
    }
}

// void vDimensionSensorRead(void * params){
//     for(;;){
//         bool result = false;
//         TickType_t startTimeChannel, maxDurationChannel, startTimeOverlap, durationOverlap = 0;
//         xSemaphoreTake(dimension_sensor_synch_sem, portMAX_DELAY);
//         startTimeChannel = xTaskGetTickCount();
//         //printf("DIMENSION SENSOR: wait overlap\n");
//         maxDurationChannel = startTimeChannel + CHANNEL_TIMEOUT;
//         while (!result && startTimeOverlap < maxDurationChannel) {
//             result = dimensionSensor.getOverlap();
//             vTaskDelay(pdMS_TO_TICKS(10UL));
//             startTimeOverlap = xTaskGetTickCount();
//         }
//         //printf("DIMENSION SENSOR: overlap\n");
//         xSemaphoreTake(stopStrainGaugeMutex, portMAX_DELAY);
//         stopStrainGauge = true;
//         xSemaphoreGive(stopStrainGaugeMutex);
//         //printf("DIMENSION SENSOR: wait no overlap\n");
//         while (result) {
//             result = dimensionSensor.getOverlap();
//             vTaskDelay(pdMS_TO_TICKS(10UL));
//         }
//         //printf("DIMENSION SENSOR: no overlap\n");
//         durationOverlap = xTaskGetTickCount() - startTimeOverlap;
//         if(durationOverlap > 135){
//             servo_2.goDegree(4); // 2 euro
//         }
//         else if(durationOverlap > 95){ //20 c
//             servo_2.goDegree(32);
//         }else if(durationOverlap > 20){
//              servo_2.goDegree(60); // 1 c
//         }
//         else{
//             servo_2.goDegree(88);
//         }
//         if(startTimeOverlap >= maxDurationChannel ){ //Overlap doesn't occur
//             printf("t0\n");
//         }else if( durationOverlap <= 10 ){ //Swings occur
//             printf("t-100\n");
//         }
//         else{
//             printf("t%d\n", durationOverlap);
//         }
//         printf("c%d\n", startTimeOverlap-startTimeChannel);
//     }
// }

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
            servo_2.goDegree(4);
        }
        else if(durationOverlap > 98){
            servo_2.goDegree(32);
        }
        else if(durationOverlap > 20){
            servo_2.goDegree(60);
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

    TaskHandle_t tStrainGauge;
     TaskHandle_t tDimensionSensor;
      TaskHandle_t tFirstServo;
       TaskHandle_t tPhotoresistor;
    
    servo_2.goDegree(4); //scrivi un costruttore alternativo per il servo per poter specificare anche l'angolo iniziale
    xTaskCreate(vPhotoresistorRead, "Entry section photoresistor's read", 1024, NULL, 3, &tPhotoresistor);
    xTaskCreate(vStrainGaugeRead, "Strain gauge's read", 1024, NULL, 1, &tStrainGauge);
    xTaskCreate(vDimensionSensorRead, "DimensionSensor's read", 1024, NULL, 3, &tDimensionSensor);
    xTaskCreate(vFirstServoAction, "Entry section blade", 1024, NULL, 2, &tFirstServo);
    
     vTaskCoreAffinitySet(tStrainGauge,3);
     vTaskCoreAffinitySet(tDimensionSensor,1);
     vTaskCoreAffinitySet(tFirstServo,3);
     vTaskCoreAffinitySet(tPhotoresistor,2);


    vTaskStartScheduler();
    
    
    return 0;
}