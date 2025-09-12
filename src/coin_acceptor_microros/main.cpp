#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
extern "C" {
#include "debug_uart.h"
#include "pico_uart_transports.h"
}

#include "header/PhotoresistorSensor.h"
#include "header/ServoActuator.h"
#include "header/StrainGaugeSensor.h"
#include "header/LM393CoupleSensor.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

/**
 * Maximum time spent waiting for the coin passage through the photocouplers.
 */
#define CHANNEL_TIMEOUT pdMS_TO_TICKS(600)

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

uint16_t lightThreshold = 0;

enum coins{
    EURO_2 = 3, 
    CENT_20 = 2,
    CENT_1 = 1,
    NOT_RECOGNIZED = 0
};

rcl_publisher_t publisher;

// photoresistor read task
void vPhotoresistorRead(void * params){
    for(;;){
        uint16_t result = photoresistor.getLight();
        if(result < lightThreshold){
            vTaskDelay(pdMS_TO_TICKS(500UL)); // wait for coin laying on the blade
            xSemaphoreGive(strainGaugeSynchSem);
            xSemaphoreGive(LM393CoupleSynchSem);
            xSemaphoreGive(bladeServoSynchSem);
            vTaskDelay(pdMS_TO_TICKS(500UL)); // wait for coin falling down
        }
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }
}

// strain gauge read task
void vStrainGaugeRead(void * params){
    uint16_t maxWeight = 0;
    for(;;){
        xSemaphoreTake(strainGaugeSynchSem, portMAX_DELAY);
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
            uart_printf("w%d\n", result); // used for calibration
            vTaskDelay(pdMS_TO_TICKS(5UL));
        }
        xQueueSend(classifierWeightQueue, &maxWeight, portMAX_DELAY);
    }
}

// photocoupler read task
void vLM393CoupleRead(void * params){
    TickType_t channelStartTime, channelDuration, overlapStartTime, overlapDuration;
    bool result = false;
    for(;;){
        xSemaphoreTake(LM393CoupleSynchSem, portMAX_DELAY);
        // new coin in the channel
        channelStartTime = xTaskGetTickCount();
        channelDuration = 0;
        overlapDuration = 0;
        while(!result && channelDuration < CHANNEL_TIMEOUT) {
            result = LM393Couple.getOverlap();
            channelDuration = xTaskGetTickCount() - channelStartTime;
            vTaskDelay(pdMS_TO_TICKS(1UL));
        }
        // stop strain gauge task
        xSemaphoreTake(stopStrainGaugeMutex, portMAX_DELAY);
        stopStrainGauge = true;
        xSemaphoreGive(stopStrainGaugeMutex);
        if (channelDuration < CHANNEL_TIMEOUT) {
            // start overlap
            overlapStartTime = xTaskGetTickCount();
            while(result){
                result = LM393Couple.getOverlap();
                vTaskDelay(pdMS_TO_TICKS(1UL));
            }
            // end overlap
            overlapDuration = xTaskGetTickCount() - overlapStartTime;
        }
        xQueueSend(classifierTimeQueue, &overlapDuration, portMAX_DELAY);
        uart_printf("t%d\n", overlapDuration); // used for calibration
        uart_printf("c%d\n",channelDuration); // used for calibration
    }
}

// blade actuation task
void vBladeServoAction(void * params){
    for(;;){
        bladeServo.goDegree(90);
        xSemaphoreTake(bladeServoSynchSem, portMAX_DELAY);
        bladeServo.goDegree(135);
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }  
}

// classifier task
void vClassifier(void * params) {
    uint16_t weight, time, degree, coinValue;
    for(;;){
        xQueueReceive(classifierWeightQueue, &weight, portMAX_DELAY);
        xQueueReceive(classifierTimeQueue, &time, portMAX_DELAY);

        // the thresholds have been found through calibration (see calibration/result folder)
        if (weight < 200 || time == 0) {
            degree = 88;
            coinValue = NOT_RECOGNIZED;
        }
        else if(time >= 110){
            degree = 4;
            coinValue = EURO_2;
        }
        else if(time >= 80){
            degree = 32;
            coinValue = CENT_20;
        }else {
             degree = 60;
             coinValue = CENT_1;
        }

        xQueueSend(slideServoQueue, &degree, portMAX_DELAY);
        xQueueSend(senderQueue, &coinValue, portMAX_DELAY); 
    }
    
}

// slide actuation task
void vSlideServoAction(void * params){
    uint16_t degree;
    slideServo.goDegree(4);
    for(;;){
        xQueueReceive(slideServoQueue, &degree, portMAX_DELAY);
        slideServo.goDegree(degree);
    }
}

// microROS publisher task
void vSender(void * params){
    uint16_t coinValue;
    std_msgs__msg__Int32 msg;
    msg.data = 0;
    for(;;){
        xQueueReceive(senderQueue, &coinValue, portMAX_DELAY);
        uart_printf("INSERTED COIN: %d\n", coinValue);
        msg.data = coinValue;
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
}

/**
 * @brief Main Task
 * This task has an extended stack and is responsible for tasks instantiation.
 * This task because sleep_ms function cannot be used.
 * @param args 
 */
void vMainTask(void * args) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init options
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	
	// create node
	rcl_node_t node;
    rcl_ret_t rc = rclc_node_init_default(&node, "publisher_node", "", &support);
    if (rc != RCL_RET_OK) {
        uart_printf("Node init failed: %d : %s\n", (int)rc, rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"coinValuePublisher"));
    
    TaskHandle_t tPhotoresistor;
    TaskHandle_t tBladeServo;
    TaskHandle_t tStrainGauge;
    TaskHandle_t tLM393Couple;
    TaskHandle_t tClassifier;
    TaskHandle_t tSlideServo;
    TaskHandle_t tSender;
    
    xTaskCreate(vPhotoresistorRead, "Entry section photoresistor's read", 1024, NULL, 3, &tPhotoresistor);
    xTaskCreate(vBladeServoAction, "Entry section blade's action", 1024, NULL, 3, &tBladeServo);
    xTaskCreate(vStrainGaugeRead, "Strain gauge's read", 1024, NULL, 4, &tStrainGauge);
    xTaskCreate(vLM393CoupleRead, "LM393 couple's read", 1024, NULL, 5, &tLM393Couple);
    xTaskCreate(vClassifier, "Classifier", 1024, NULL, 4, &tClassifier);
    xTaskCreate(vSlideServoAction, "Final section slide's action", 1024, NULL, 4, &tSlideServo);
    xTaskCreate(vSender, "Data sender", 1024, NULL, 2, &tSender);

    // stop task indefinitely
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(10000));  // Yield to other tasks
	}
}

int main(){
    uint16_t baseLightValue = photoresistor.getLight();
    lightThreshold = baseLightValue * 0.6;
    initialize_debug_uart();

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_usb_transport_open,
		pico_usb_transport_close,
		pico_usb_transport_write,
		pico_usb_transport_read
	);

    TaskHandle_t mainTask;

    xTaskCreate(vMainTask, "Main Task", 5000, NULL, 1, &mainTask);
    // Bind the task to a single core
	vTaskCoreAffinitySet(mainTask, 1);
    
    vTaskStartScheduler();
    return 0;
}