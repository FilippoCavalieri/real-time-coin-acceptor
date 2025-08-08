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

QueueHandle_t classifier_weight_queue = xQueueCreate(1, sizeof(uint16_t));
QueueHandle_t classifier_time_queue = xQueueCreate(1, sizeof(uint16_t));
QueueHandle_t second_servo_queue = xQueueCreate(1, sizeof(uint16_t));
QueueHandle_t sender_queue = xQueueCreate(1, sizeof(uint16_t));

bool stopStrainGauge = false;
SemaphoreHandle_t stopStrainGaugeMutex = xSemaphoreCreateMutex();

enum coins{
    EURO_2, 
    CENT_20,
    CENT_1,
    NOT_RECOGNIZED
};

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
            //vTaskDelay(pdMS_TO_TICKS(5UL));
        }
        printf("STRAIN GAUGE: stop\n");
    }
}


void dimensionSensorCalibration(){

}

void vDimensionSensorRead(void * params){
    for(;;){
        xSemaphoreTake(dimension_sensor_synch_sem, portMAX_DELAY);
        startTimeChannel = xTaskGetTickCount();
        //printf("DIMENSION SENSOR: wait overlap\n");
        maxDurationChannel = startTimeChannel + CHANNEL_TIMEOUT;
        while (!result && startTimeOverlap < maxDurationChannel) {
        TickType_t startTimeChannel = xTaskGetTickCount();
        TickType_t durationChannel = 0;
        bool result = false;
        printf("DIMENSION SENSOR: wait overlap\n");
        while (!result && durationChannel < CHANNEL_TIMEOUT) {
            result = dimensionSensor.getOverlap();
            //vTaskDelay(pdMS_TO_TICKS(1UL));
            durationChannel = xTaskGetTickCount() - startTimeChannel;
        }
        //startTimeOverlap = xTaskGetTickCount();
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
        if(durationChannel >= maxDurationChannel ){ //Overlap doesn't occur
            printf("t0\n");
        }else if( durationOverlap <= 10 ){ //Swings occur
            printf("t-100\n");
        }
        else{
            printf("t%d\n", durationOverlap);
        }
        printf("c%d\n", durationChannel);
    }
}

void vFirstServoAction(void * params){
    servo_1.goDegree(90);
    for(;;){
        servo_1.goDegree(90);
        xSemaphoreTake(first_servo_synch_sem, portMAX_DELAY);
        servo_1.goDegree(135);
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }  
}

void vClassifier(void * params){
    uint16_t weight, time, degree, coinValue;
    for(;;){
        xQueueReceive(classifier_weight_queue, &weight, portMAX_DELAY);
        xQueueReceive(classifier_time_queue, &time, portMAX_DELAY);

        if(time > 125){ // 2 euro
            degree = 4;
            coinValue = EURO_2;
        }
        else if(time > 90){ //20 c
            degree = 32;
            coinValue = CENT_20;
        }else if(time > 0){ // 1 c
             degree = 60;
             coinValue = CENT_1;
        }
        else{ //not recognized
            degree = 88;
            coinValue = NOT_RECOGNIZED;
        }

        xQueueSend(second_servo_queue, &degree, portMAX_DELAY);
        xQueueSend(sender_queue, &coinValue, portMAX_DELAY); 
    }
    
}

void vSecondServoAction(void * params){
    uint16_t degree;
    servo_2.goDegree(4);
    for(;;){
        xQueueReceive(second_servo_queue, &degree, portMAX_DELAY);
        servo_2.goDegree(degree);
    }
}

void vSender(void * params){
    uint16_t coinValue;
    for(;;){
        xQueueReceive(sender_queue, &coinValue, portMAX_DELAY);
        printf("INSERTED COIN: %d\n", coinValue);
    }
}

void initialize_board(){
    stdio_init_all();
}

int main(){
    initialize_board();
    xTaskCreate(vPhotoresistorRead, "Entry section photoresistor's read", 1024, NULL, 10, NULL);
    xTaskCreate(vStrainGaugeRead, "Strain gauge's read", 1024, NULL, 8, NULL);
    xTaskCreate(vDimensionSensorRead, "DimensionSensor's read", 1024, NULL, 7, NULL);
    xTaskCreate(vFirstServoAction, "Entry section blade", 1024, NULL, 9, NULL);

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