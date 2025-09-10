#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "header/PhotoresistorSensor.h"
#include "header/ServoActuator.h"
#include "header/StrainGaugeSensor.h"
#include "header/LM393CoupleSensor.h"

#define CHANNEL_TIMEOUT 6000

PhotoresistorSensor photoresistor(26);
ServoActuator bladeServo(19);
StrainGaugeSensor strainGauge(27);
ServoActuator slideServo(21); 
LM393CoupleSensor LM393Couple(10, 12);

SemaphoreHandle_t strainGaugeSynchSem = xSemaphoreCreateBinary();
SemaphoreHandle_t LM393CoupleSynchSem = xSemaphoreCreateBinary();
SemaphoreHandle_t bladeServoSynchSem = xSemaphoreCreateBinary();

QueueHandle_t classifierWeightQueue = xQueueCreate(1, sizeof(uint16_t));
QueueHandle_t classifierTimeQueue = xQueueCreate(1, sizeof(uint16_t));
QueueHandle_t slideServoQueue = xQueueCreate(1, sizeof(uint16_t));
QueueHandle_t senderQueue = xQueueCreate(1, sizeof(uint16_t));

bool stopStrainGauge = false;
SemaphoreHandle_t stopStrainGaugeMutex = xSemaphoreCreateMutex();

uint16_t light_threshold = 0;

enum coins{
    EURO_2, 
    CENT_20,
    CENT_1,
    NOT_RECOGNIZED
};

void vPhotoresistorRead(void * params){
    for(;;){
        uint16_t result = photoresistor.getLight();
        printf("PHOTORESISTOR: Read value: %d\n", result, result);
        if(result < light_threshold){
            vTaskDelay(pdMS_TO_TICKS(500UL)); //wait for coin lay on the blade
            xSemaphoreGive(strainGaugeSynchSem);
            xSemaphoreGive(LM393CoupleSynchSem);
            xSemaphoreGive(bladeServoSynchSem);
            vTaskDelay(pdMS_TO_TICKS(500UL)); //wait for coin fall down
        }
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }
}

void vStrainGaugeRead(void * params){
    uint16_t maxWeight = 0;
    for(;;){
        xSemaphoreTake(strainGaugeSynchSem, portMAX_DELAY);
        //printf("STRAIN GAUGE: start\n");
        for(;;){
            xSemaphoreTake(stopStrainGaugeMutex, portMAX_DELAY);
            if(stopStrainGauge){
                stopStrainGauge = false;
                xSemaphoreGive(stopStrainGaugeMutex);
                break;
            }
            xSemaphoreGive(stopStrainGaugeMutex);
            uint16_t result = strainGauge.getWeight();
            if(result > maxWeight){
                maxWeight = result;
            }
            printf("w%d\n", result);
            vTaskDelay(pdMS_TO_TICKS(5UL));
        }
        xQueueSend(classifierWeightQueue, &maxWeight, portMAX_DELAY); //?
        //printf("STRAIN GAUGE: stop\n");
    }
}

void dimensionSensorCalibration(){

}

void vLM393CoupleRead(void * params){
    TickType_t channelStartTime, channelDuration, overlapStartTime, overlapDuration = 0;
    bool result = false;
    for(;;){
        xSemaphoreTake(LM393CoupleSynchSem, portMAX_DELAY);
        channelStartTime = xTaskGetTickCount();
        channelDuration = 0;
        while(!result && channelDuration < CHANNEL_TIMEOUT) {
            result = LM393Couple.getOverlap();
            channelDuration = xTaskGetTickCount() - channelStartTime;
            vTaskDelay(pdMS_TO_TICKS(1UL));
        }
        overlapStartTime = xTaskGetTickCount();
        //printf("DIMENSION SENSOR: overlap\n");
        xSemaphoreTake(stopStrainGaugeMutex, portMAX_DELAY);
        stopStrainGauge = true;
        xSemaphoreGive(stopStrainGaugeMutex);
        while(result){
            result = LM393Couple.getOverlap();
            vTaskDelay(pdMS_TO_TICKS(1UL));
        }
        //printf("DIMENSION SENSOR: no overlap\n");
        overlapDuration = xTaskGetTickCount() - overlapStartTime;
        xQueueSend(classifierTimeQueue, &overlapDuration, portMAX_DELAY);
        if(channelDuration >= CHANNEL_TIMEOUT ){ //Overlap doesn't occur
            printf("t0\n");
        }else if(overlapDuration <= 10){ //Swings occur
            printf("t-100\n");
        }
        else{
            //uart_printf("t%d\n", overlapDuration);
            printf("t%u\n", overlapDuration);
        }
        //uart_printf("c%d\n", durationChannel);
        printf("c%u\n", channelDuration);
    }
}

void vBladeServoAction(void * params){
    for(;;){
        bladeServo.goDegree(90);
        xSemaphoreTake(bladeServoSynchSem, portMAX_DELAY);
        printf("ACTION\n");
        bladeServo.goDegree(135);
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }  
}

void vClassifier(void * params){
    uint16_t weight, time, degree, coinValue;
    for(;;){
        xQueueReceive(classifierWeightQueue, &weight, portMAX_DELAY);
        xQueueReceive(classifierTimeQueue, &time, portMAX_DELAY);

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

        xQueueSend(slideServoQueue, &degree, portMAX_DELAY);
    }
    
}

void vSlideServoAction(void * params){
    uint16_t degree;
    slideServo.goDegree(4);
    for(;;){
        xQueueReceive(slideServoQueue, &degree, portMAX_DELAY);
        slideServo.goDegree(degree);
    }
}

void initialize_board(){
    stdio_init_all();
    uint16_t base_light_value = photoresistor.getLight();
    light_threshold = base_light_value * 0.6;
}

int main(){
    initialize_board();
    
    TaskHandle_t tPhotoresistor;
    TaskHandle_t tBladeServo;
    TaskHandle_t tStrainGauge;
    TaskHandle_t tLM393Couple;
    TaskHandle_t tClassifier;
    TaskHandle_t tSlideServo;
    TaskHandle_t tSender;
    
    
    xTaskCreate(vPhotoresistorRead, "Entry section photoresistor's read", 1024, NULL, 2, &tPhotoresistor);
    xTaskCreate(vBladeServoAction, "Entry section blade's action", 1024, NULL, 2, &tBladeServo);
    xTaskCreate(vStrainGaugeRead, "Strain gauge's read", 1024, NULL, 3, &tStrainGauge);
    xTaskCreate(vLM393CoupleRead, "LM393 couple's read", 1024, NULL, 4, &tLM393Couple);
    xTaskCreate(vClassifier, "Classifier", 1024, NULL, 3, &tClassifier);
    xTaskCreate(vSlideServoAction, "Final section slide's action", 1024, NULL, 3, &tSlideServo);    

    vTaskStartScheduler();
    return 0;
}