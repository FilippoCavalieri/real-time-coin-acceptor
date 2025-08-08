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

Fotoresistore fotoresistore(26);
Estensimetro estensimetro(27);
Servo servo_1(19); //CAMBIA
Servo servo_2(21); //CAMBIA

SemaphoreHandle_t strain_gauge_synch_sem = xSemaphoreCreateBinary();
SemaphoreHandle_t speed_sensor_synch_sem = xSemaphoreCreateBinary();
SemaphoreHandle_t first_servo_synch_sem = xSemaphoreCreateBinary();
SemaphoreHandle_t second_servo_synch_sem_1 = xSemaphoreCreateBinary();
SemaphoreHandle_t second_servo_synch_sem_2 = xSemaphoreCreateBinary();



uint16_t max_weight = 0;
uint16_t overlap_time = 0;

void vPhotoresistorRead(void * params){
    int already_signaled = 0;
    for(;;){
        uint16_t result = fotoresistore.getLight();
        //printf("PHOTORESISTOR: Read value: hex: 0x%03x, dec: %d\n", result, result);
        if(result < 1000 && !already_signaled){
            already_signaled = 1;
            xSemaphoreGive(strain_gauge_synch_sem);
            xSemaphoreGive(first_servo_synch_sem);
            vTaskDelay(pdMS_TO_TICKS(100UL));
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
        counter = 0;
        xSemaphoreTake(strain_gauge_synch_sem, portMAX_DELAY);
        for(int i=0; i < 100; i++){
            uint16_t result = estensimetro.getWeight();
            if(result > 100){
                printf("STRAIN GAUGE: value: hex: 0x%03x, dec: %d\n", result, result);
                //
                counter++;
            }
            if(counter == 5){
                xSemaphoreGive(second_servo_synch_sem_1);
            }
            vTaskDelay(pdMS_TO_TICKS(1UL));
        }
    
        printf("STRAIN GAUGE: stop\n");
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }
}


//TO-DO
void vSpeedSensorRead(void * params){
    for(;;){
        xSemaphoreTake(speed_sensor_synch_sem, portMAX_DELAY);
        for(int i=0; i < 100; i++){
            uint16_t result = estensimetro.getWeight();
            if(result > 100){
                printf("STRAIN GAUGE: value: hex: 0x%03x, dec: %d\n", result, result);
            }
            vTaskDelay(pdMS_TO_TICKS(1UL));
        }
        printf("STRAIN GAUGE: stop\n");
        vTaskDelay(pdMS_TO_TICKS(100UL));
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


void vSecondServoAction(void * params){
    int stack_degrees[4] = {4, 32, 60, 88};
    int counter = 0;
    servo_2.goDegree(4);
    for(;;){
        xSemaphoreTake(second_servo_synch_sem_1, portMAX_DELAY);
        //xSemaphoreTake(second_servo_synch_sem_2, portMAX_DELAY);
        //CONTROLLO
        servo_2.goDegree(stack_degrees[counter++ % 4]);
        vTaskDelay(pdMS_TO_TICKS(100UL));
    }  
}

void initialize_board(){
    stdio_init_all();
}

int main(){
    initialize_board();
    xTaskCreate(vPhotoresistorRead, "Entry section photoresistor's read", 1024, NULL, 1, NULL);
    xTaskCreate(vStrainGaugeRead, "Strain gauge's read", 1024, NULL, 1, NULL);
    xTaskCreate(vFirstServoAction, "Entry section blade", 1024, NULL, 1, NULL);
    xTaskCreate(vSecondServoAction, "Final section slide", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
    return 0;
}