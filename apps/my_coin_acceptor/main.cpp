#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

// #include <rcl/rcl.h>
// #include <std_msgs/msg/int32.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <rmw_microros/rmw_microros.h>
// extern "C" {
// #include "debug_uart.h"
// #include "pico_uart_transports.h"
// }

#include "header/PhotoresistorSensor.h"
#include "header/ServoActuator.h"
#include "header/StrainGaugeSensor.h"
#include "header/LM393CoupleSensor.h"

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

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

enum coins{
    EURO_2, 
    CENT_20,
    CENT_1,
    NOT_RECOGNIZED
};

// rcl_publisher_t publisher;

void vPhotoresistorRead(void * params){
    for(;;){
        uint16_t result = photoresistor.getLight();
        printf("PHOTORESISTOR: Read value: %d\n", result, result);
        if(result < 1000){
            vTaskDelay(pdMS_TO_TICKS(500UL)); //wait for coin fall on the blade
            xSemaphoreGive(strainGaugeSynchSem);
            xSemaphoreGive(LM393CoupleSynchSem);
            xSemaphoreGive(bladeServoSynchSem);
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
            //uart_printf("t0\n");
            printf("t0\n");
        }else if(overlapDuration <= 10){ //Swings occur
            //uart_printf("t-100\n");
            printf("t-100\n");
        }
        else{
            //uart_printf("t%d\n", overlapDuration);
            printf("t%d\n", overlapDuration);
        }
        //uart_printf("c%d\n", durationChannel);
        printf("c%d\n", channelDuration);
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
        //xQueueSend(senderQueue, &coinValue, portMAX_DELAY); 
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

// void vSender(void * params){
//     uint16_t coinValue;
//     std_msgs__msg__Int32 msg;
//     msg.data = 0;
//     for(;;){
//         xQueueReceive(senderQueue, &coinValue, portMAX_DELAY);
//         msg.data = coinValue;
//         RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//         uart_printf("INSERTED COIN: %d\n", coinValue);
//     }
// }

void initialize_board(){
    stdio_init_all();
    //initialize_debug_uart();
}

// void initialize_micro_ros(){
//     rmw_uros_set_custom_transport(
// 		true,
// 		NULL,
// 		pico_serial_transport_open,
// 		pico_serial_transport_close,
// 		pico_serial_transport_write,
// 		pico_serial_transport_read
// 	);

//     rcl_allocator_t allocator = rcl_get_default_allocator();
// 	rclc_support_t support;

// 	// create init options
// 	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
// 	RCCHECK(rcl_init_options_init(&init_options, allocator));
// 	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	
// 	// create node
// 	rcl_node_t node;
// 	RCCHECK(rclc_node_init_default(&node, "publisher_node", "", &support));

// 	// create publisher
// 	RCCHECK(rclc_publisher_init_default(
// 		&publisher,
// 		&node,
// 		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
// 		"coinValuePublisher"));
// }

int main(){
    initialize_board();
    //initialize_micro_ros();
    
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
    //xTaskCreate(vSender, "Entry section blade", 1024, NULL, 1, &tSender);
    
    
    //vTaskCoreAffinitySet(tStrainGauge,1);
    //vTaskCoreAffinitySet(tLM393Couple,2);
    //vTaskCoreAffinitySet(tBladeServo,3);
    //vTaskCoreAffinitySet(tPhotoresistor,2);
    


    vTaskStartScheduler();
    return 0;
}