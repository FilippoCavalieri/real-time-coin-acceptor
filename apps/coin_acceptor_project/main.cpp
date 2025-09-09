#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
extern "C" {
#include "debug_uart.h"
#include "pico_uart_transports.h"
}

#include "Fotoresistore.h"
#include "Estensimetro.h"
#include "Servo.h"
#include "DimensionSensor.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

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

rcl_publisher_t publisher;

void vPhotoresistorRead(void * params){
    int already_signaled = 0;
    for(;;){
        uint16_t result = fotoresistore.getLight();
        //printf("PHOTORESISTOR: Read value: %d\n", result, result);
        if(result < 1000){
            vTaskDelay(pdMS_TO_TICKS(500UL));
            xSemaphoreGive(strain_gauge_synch_sem);
            xSemaphoreGive(dimension_sensor_synch_sem);
            xSemaphoreGive(first_servo_synch_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }
}

void vStrainGaugeRead(void * params){
    uint16_t maxWeight = 0;
    int counter;
    for(;;){
        xSemaphoreTake(strain_gauge_synch_sem, portMAX_DELAY);
        //printf("STRAIN GAUGE: start\n");
        for(;;){
            xSemaphoreTake(stopStrainGaugeMutex, portMAX_DELAY);
            if (stopStrainGauge) {
                stopStrainGauge = false;
                xSemaphoreGive(stopStrainGaugeMutex);
                break;
            }
            xSemaphoreGive(stopStrainGaugeMutex);
            uint16_t result = estensimetro.getWeight();
            if(result > maxWeight){
                maxWeight = result;
            }
            //printf("w%d\n", result);
            vTaskDelay(pdMS_TO_TICKS(1UL));
        }
        xQueueSend(classifier_weight_queue, &maxWeight, portMAX_DELAY);
        //printf("STRAIN GAUGE: stop\n");
    }
}


void dimensionSensorCalibration(){

}

void vDimensionSensorRead(void * params){
    for(;;){
        xSemaphoreTake(dimension_sensor_synch_sem, portMAX_DELAY);
        TickType_t startTimeChannel = xTaskGetTickCount();
        TickType_t durationChannel = 0;
        bool result = false;
        while (!result && durationChannel < CHANNEL_TIMEOUT) {
            result = dimensionSensor.getOverlap();
            durationChannel = xTaskGetTickCount() - startTimeChannel;
            vTaskDelay(pdMS_TO_TICKS(1UL));
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
            vTaskDelay(pdMS_TO_TICKS(1UL));
        }
        //printf("DIMENSION SENSOR: no overlap\n");
        TickType_t durationOverlap = xTaskGetTickCount() - startTimeOverlap;
        xQueueSend(classifier_time_queue, &durationOverlap, portMAX_DELAY);
        if(durationChannel >= CHANNEL_TIMEOUT ){ //Overlap doesn't occur
            uart_printf("t0\n");
        }else if( durationOverlap <= 10 ){ //Swings occur
            uart_printf("t-100\n");
        }
        else{
            uart_printf("t%d\n", durationOverlap);
        }
        uart_printf("c%d\n", durationChannel);
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
    std_msgs__msg__Int32 msg;
    msg.data = 0;
    for(;;){
        xQueueReceive(sender_queue, &coinValue, portMAX_DELAY);
        msg.data = coinValue;
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        uart_printf("INSERTED COIN: %d\n", coinValue);
    }
}

void initialize_board(){
    //stdio_init_all();
    initialize_debug_uart();
}

void initialize_micro_ros(){
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init options
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	
	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "publisher_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"coinValuePublisher"));
}

int main(){
    initialize_board();
    initialize_micro_ros();
    
    TaskHandle_t tStrainGauge;
    TaskHandle_t tDimensionSensor;
    TaskHandle_t tFirstServo;
    TaskHandle_t tPhotoresistor;
    TaskHandle_t tClassifier;
    TaskHandle_t tSecondServo;
    TaskHandle_t tSender;
    
    
    xTaskCreate(vPhotoresistorRead, "Entry section photoresistor's read", 1024, NULL, 2, &tPhotoresistor);
    xTaskCreate(vStrainGaugeRead, "Strain gauge's read", 1024, NULL, 3, &tStrainGauge);
    xTaskCreate(vDimensionSensorRead, "DimensionSensor's read", 1024, NULL, 3, &tDimensionSensor);
    xTaskCreate(vFirstServoAction, "Entry section blade", 1024, NULL, 2, &tFirstServo);
    xTaskCreate(vClassifier, "Classifier", 1024, NULL, 3, &tClassifier);
    xTaskCreate(vSecondServoAction, "Final section slide", 1024, NULL, 3, &tSecondServo);
    xTaskCreate(vSender, "Entry section blade", 1024, NULL, 1, &tSender);
    
    /*
    vTaskCoreAffinitySet(tStrainGauge,3);
    vTaskCoreAffinitySet(tDimensionSensor,1);
    vTaskCoreAffinitySet(tFirstServo,3);
    vTaskCoreAffinitySet(tPhotoresistor,2);
    */


    vTaskStartScheduler();
    return 0;
}