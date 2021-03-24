


#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define GPIO_OUTPUT_1 18
#define GPIO_OUTPUT_2 19
#define GPIO_OUTPUT_SEL ((1ULL << GPIO_OUTPUT_1)|(1ULL << GPIO_OUTPUT_2))
#define GPIO_INPUT_1 4 // Interrupt at anyedge
#define GPIO_INPUT_2 5 // Interrupt at positive edge
#define GPIO_INPUT_SEL ((1ULL << GPIO_INPUT_1) | (1ULL << GPIO_INPUT_2))
#define ESP_INTR_FLAG_DEFAULT 0

void task_1(void *args);
void task_2(void *args);


gpio_config_t io_conf;
static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void * args){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}





void app_main() {
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_OUTPUT_SEL;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0; 
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_intr_type(GPIO_INPUT_1,GPIO_INTR_ANYEDGE);
    gpio_evt_queue = xQueueCreate(10,sizeof(uint32_t));
    xTaskCreate(task_1,"task_1_interrupt_handling",1024,NULL,3,NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_1,gpio_isr_handler,(void*) GPIO_INPUT_1);
    gpio_isr_handler_add(GPIO_INPUT_2,gpio_isr_handler,(void*) GPIO_INPUT_2);
    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    xTaskCreate(task_2,"blinking task",1024,NULL,3,NULL);






}


void task_1(void *args){
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
    vTaskDelete(NULL);
    
}

void task_1(void *args){
    int cnt = 0;
    while(1){
        printf("cnt is %d",cnt++);
        gpio_set_level(GPIO_OUTPUT_1,cnt % 2);
        gpio_set_level(GPIO_OUTPUT_2,cnt % 2);
        vTaskDelay(1000/ portTICK_RATE_MS);
    }
    
}

