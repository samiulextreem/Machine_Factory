#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"



#define INSTR_CUT_AUTO 1
#define INSTR_CUT_USER  2
#define INSTR_CUT_STOP 3
#define INSTR_ROLLER_AUTO 1
#define INSTR_ROLLER_USER 2
#define INSTR_ROLLER_STOP 3
#define CUTSTPR_DEPTH_VARIABLE 15 // How much deep the blade will go down
#define CUTSTPR_SPEED_VARIABLE 20 //in ms
#define CUTSTPR_ENABLE_DELAY 2 // in ms
#define ROLLSTPR_DEPTH_VARIABLE 100 // how much the motor have to rotate to pass the bag
#define ROLLSTPR_SPEED_VARIABLE 20
#define ROLLSTPR_ENABLE_DELAY 2 // in ms

#define BUF_SIZE (1024)

#define ROLLSTPR_STEP_OUTPUT 22
#define ROLLSTPR_DIR_OUTPUT 13
#define ROLLSTPR_EN_OUTPUT 14
#define CUTSTPR_STEP_OUTPUT 4
#define CUTSTPR_DIR_OUTPUT 26
#define CUTSTPR_EN_OUTPUT 27
#define GPIO_OUTPUT_SEL ((1ULL << ROLLSTPR_STEP_OUTPUT)|(1ULL << ROLLSTPR_DIR_OUTPUT) | (1ULL << ROLLSTPR_EN_OUTPUT) | (1ULL << CUTSTPR_STEP_OUTPUT)|(1ULL << CUTSTPR_DIR_OUTPUT) | (1ULL << CUTSTPR_EN_OUTPUT))
#define ALL_HOLD_INPUT 32 // Interrupt at positive edge stop
#define USER_RUN_INPUT 33 // Interrupt at positive edge
#define USER_CUT_INPUT 34 // Interrupt at positive edge
#define ALL_RUN_INPUT  35 // Interrupt at positive edge
#define GPIO_INPUT_SEL ((1ULL << ALL_HOLD_INPUT) | (1ULL << USER_RUN_INPUT) | (1ULL << USER_CUT_INPUT) | (1ULL << ALL_RUN_INPUT))
#define ESP_INTR_FLAG_DEFAULT 0


//main operational task by stepper
void task_1_cutter(void *args);
void task_2_roller(void *args);

//input tasks
void task_3_user_all_hold_input(void *args);
void task_4_user_cut_input(void *args);
void task_5_user_roll_input(void *args);
void task_6_user_all_run_input(void *args);
void task_7_uart_listener(void *args);

static gpio_config_t io_conf;
static xQueueHandle cutter_inctruction_que = NULL;
static xQueueHandle roller_instruction_que = NULL;
static xQueueHandle uart_instruction_que_all_stop = NULL;
static xQueueHandle uart_instruction_que_all_run = NULL;
static xQueueHandle uart_instruction_que_cutter_run = NULL;
static xQueueHandle uart_instruction_que_roller_run = NULL;

static xTaskHandle task_1_cutter_handle;
static xTaskHandle task_2_roller_handle;
static xTaskHandle task_3_user_all_stop_handle;
static xTaskHandle task_4_user_cut_handle;
static xTaskHandle task_5_user_roll_handle;
static xTaskHandle task_6_user_all_run_handle;

eTaskState task_1_state;
eTaskState task_2_state;




void app_main() {
    //setting up outputs
    uint8_t roller_bootup_start;
    roller_bootup_start = INSTR_ROLLER_AUTO;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_OUTPUT_SEL;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0; 
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    //seting up inputs
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);



    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    

    // Install UART driver using an event queue here
    printf("\n");
    printf("\n");
    printf("\n");
    printf("\n");
    printf("\n");

    //char *test_str = "This is a test string.\n";
    //uart_write_bytes(UART_NUM_0, (const char*)test_str, strlen(test_str)); // testing if can write data to uart 0
    

  




    cutter_inctruction_que = xQueueCreate(10,sizeof(uint8_t));
    roller_instruction_que = xQueueCreate(10,sizeof(uint8_t));
    uart_instruction_que_all_stop = xQueueCreate(10,sizeof(uint8_t));
    uart_instruction_que_all_run = xQueueCreate(10,sizeof(uint8_t));
    uart_instruction_que_cutter_run = xQueueCreate(10,sizeof(uint8_t));
    uart_instruction_que_roller_run = xQueueCreate(10,sizeof(uint8_t));
    /*
        Have to write a function block which will position the cutter motor
    */


    
    xTaskCreate(task_1_cutter,"function for stepper cutter",2024,NULL,3,&task_1_cutter_handle);
    xTaskCreate(task_2_roller,"function for stepper roller",2024,NULL,3,&task_2_roller_handle);
    xTaskCreate(task_3_user_all_hold_input, "function for all stop command", 2024, NULL, 3, &task_3_user_all_stop_handle);
    xTaskCreate(task_4_user_cut_input,"handle user cut input functio",2024,NULL,3,&task_4_user_cut_handle);
    xTaskCreate(task_5_user_roll_input,"handle user roll input functio",2024,NULL,3,&task_5_user_roll_handle);
    xTaskCreate(task_6_user_all_run_input,"handle user all input functio",2024,NULL,3,&task_6_user_all_run_handle);
    xTaskCreate(task_7_uart_listener, "function for getting command from PC", 2024, NULL, 3, NULL);

    if(xQueueSend(roller_instruction_que,&roller_bootup_start,10)){
        printf("starting the roller at startup\n");
    }
    task_1_state = eTaskGetState(&task_1_cutter_handle);
    task_2_state = eTaskGetState(&task_2_roller_handle);
    printf("cutter state is %d\n", task_1_state);
    printf("roller state is %d\n",task_2_state);
}

void task_1_cutter(void *args){
    //if revieved 1 then  proceed to cut cycle by auto
    //if recieved 2 then  proceed to cut cycle by user
    //if recieved 0 then  stop cycle
    uint8_t instr_num;
    uint8_t roller_signal = INSTR_ROLLER_AUTO;
    uint8_t roller_message_copy;
    for (;;)
    {
        if(xQueueReceive(cutter_inctruction_que, &instr_num, 10) == pdTRUE) {
            printf("cutter : recieved instruction %d\n", instr_num);
            gpio_set_level(CUTSTPR_EN_OUTPUT,1); //enabling the cutter stepper driver
            vTaskDelay(CUTSTPR_ENABLE_DELAY/portTICK_RATE_MS); // delaying small amount before pwm produces
            if (instr_num == INSTR_CUT_AUTO ){
                //vTaskSuspend(task_2_roller_handle);
                printf("cutter : proceed to cut ---- auto instruction\n");
                //run the cycle
                gpio_set_level(CUTSTPR_DIR_OUTPUT,1); // setting the direction of cutter stepper
                for (int x = 0; x < CUTSTPR_DEPTH_VARIABLE ; x++ ){
                    gpio_set_level(CUTSTPR_STEP_OUTPUT,1);
                    vTaskDelay(CUTSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    gpio_set_level(CUTSTPR_STEP_OUTPUT,0);
                    vTaskDelay(CUTSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    printf("                                                                                 cutter : pressing down  loop remaining %d\n", (CUTSTPR_DEPTH_VARIABLE - x));
                }
                gpio_set_level(CUTSTPR_DIR_OUTPUT,0); // reversing the direction of cutter stepper
                for (int x = 0; x < CUTSTPR_DEPTH_VARIABLE ; x++ ){
                    gpio_set_level(CUTSTPR_STEP_OUTPUT,1);
                    vTaskDelay(CUTSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    gpio_set_level(CUTSTPR_STEP_OUTPUT,0);
                    vTaskDelay(CUTSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    printf("                                                                                 cutter : pressing up  loop remaining %d\n", (CUTSTPR_DEPTH_VARIABLE - x));
                }
                gpio_set_level(CUTSTPR_EN_OUTPUT,0); //disabling the cutter stepper driver
                printf("cutter : Finished\n");
                if(xQueueSend(roller_instruction_que,&roller_signal,5) == pdTRUE){
                        printf("cutter : sent instruction to the roller 1 and will be suspend self soon \n");
                        vTaskDelay(5); // delaying small amount before suspend
                        vTaskSuspend(NULL);
                }
                else{
                    printf("cutter : failed to sent roller instruction 1 \n");
                    printf("########################  CRITICAL ERROR   ########################");
                    vTaskSuspendAll();
                }
    
                
            }
            else if (instr_num == INSTR_CUT_USER){
                printf("proceed to cut ---- user instruction\n");
                //run the cycle 
                gpio_set_level(CUTSTPR_DIR_OUTPUT,1); // setting the direction of cutter stepper
                for (int x = 0; x < CUTSTPR_DEPTH_VARIABLE ; x++ ){
                    gpio_set_level(CUTSTPR_STEP_OUTPUT,1);
                    vTaskDelay(CUTSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    gpio_set_level(CUTSTPR_STEP_OUTPUT,0);
                    vTaskDelay(CUTSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                }
                gpio_set_level(CUTSTPR_DIR_OUTPUT,0); // reversing the direction of cutter stepper
                for (int x = 0; x < CUTSTPR_DEPTH_VARIABLE ; x++ ){
                    gpio_set_level(CUTSTPR_STEP_OUTPUT,1);
                    vTaskDelay(CUTSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    gpio_set_level(CUTSTPR_STEP_OUTPUT,0);
                    vTaskDelay(CUTSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                }
                while(1){
                    if(xQueueSend(roller_instruction_que,INSTR_ROLLER_AUTO,10) != pdTRUE){
                        printf("ERROR ROLLER DID NOT RECIEVE INSTR_ROLLER_USER\n");
                    }
                    else if(xQueueSend(roller_instruction_que,INSTR_ROLLER_AUTO,10) == pdTRUE){
                        printf("send to queue INSTR_ROLLER_USER\n");
                        break;
                    }
                }

            }

        }
        vTaskDelay(5);
    }
    vTaskDelete(NULL);
    
}


void task_2_roller(void *args){
    //if revieved 1 then  proceed to cut cycle by auto
    //if recieved 2 then  proceed to cut cycle by user
    //if recieved 0 then  stop cycle
    gpio_set_level(ROLLSTPR_DIR_OUTPUT, 1);
    uint8_t instr_num;
    uint8_t cutter_signal = INSTR_CUT_AUTO;
    for (;;)
    {
        if(xQueueReceive(roller_instruction_que, &instr_num , 10) == pdTRUE){
            printf("roller : recieved instruction %d\n", instr_num);
            if(instr_num == INSTR_ROLLER_STOP){
                printf("roller : recieved command to suspend self\n");
                vTaskSuspend(NULL);
            }
            gpio_set_level(ROLLSTPR_EN_OUTPUT,1);
            vTaskDelay(CUTSTPR_ENABLE_DELAY/portTICK_RATE_MS); // delaying small amount before pwm produces
            if(instr_num == INSTR_ROLLER_AUTO || instr_num == 114){
                //vTaskSuspend(task_1_cutter_handle);
                printf("roller : proceed to roll ---- auto instruction\n");
                gpio_set_level(ROLLSTPR_EN_OUTPUT,1);
                for(int x = 0; x < ROLLSTPR_DEPTH_VARIABLE ; x++){
                    gpio_set_level(ROLLSTPR_STEP_OUTPUT,1);
                    vTaskDelay(ROLLSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    gpio_set_level(ROLLSTPR_STEP_OUTPUT,0);
                    vTaskDelay(ROLLSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    printf("                                                                              roller : rolling by auto_instruction loop remaining %d\n",(ROLLSTPR_DEPTH_VARIABLE-x));
                }
                gpio_set_level(ROLLSTPR_EN_OUTPUT,0); //disabling the cutter stepper driver
                printf("roller : Finished\n");
                if(xQueueSend(cutter_inctruction_que,&cutter_signal,5)){
                    printf("roller : sent instruction to the cutter  1 \n");
                    vTaskResume(task_1_cutter_handle);
                    vTaskDelay(2); // delaying small amount before passing message
                }
                else{
                    printf("roller : failed to sent cutter instruction 1\n");
                    printf("##################### CRITICAL ERROR #########################3");
                    vTaskSuspendAll();
                }
            }
            else if(instr_num == INSTR_ROLLER_USER){
                printf("roller : proceed to roll ---- user instruction\n");
                gpio_set_level(ROLLSTPR_EN_OUTPUT,1);
                for(int x =0; x < ROLLSTPR_DEPTH_VARIABLE ; x++){
                    gpio_set_level(ROLLSTPR_STEP_OUTPUT,1);
                    vTaskDelay(ROLLSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    gpio_set_level(ROLLSTPR_STEP_OUTPUT,0);
                    vTaskDelay(ROLLSTPR_SPEED_VARIABLE/portTICK_RATE_MS);
                    printf("roller : rolling by user_instruction loop remaining %d \n",x);
                }
                instr_num = NULL;
            }
            

        }
        vTaskDelay(2);
    }
}


void task_3_user_all_hold_input(void *args){
    uint8_t  data_all_hold_input;
    uint8_t  roller_signal_stop = INSTR_ROLLER_STOP;
    while (1){

        if (gpio_get_level(ALL_HOLD_INPUT) == 1){

            printf("task #3 : recieved command for all hold input\n");
        }
        if (xQueueReceive(uart_instruction_que_all_stop, &data_all_hold_input, 5) == pdTRUE){
            vTaskDelay(2);
            printf("task #3 : recieved item is '%c'\n", data_all_hold_input);
            if (eTaskGetState(task_2_roller_handle) == eSuspended){
                printf("task #3 : roller is already suspended\n");
                vTaskDelay(10);
            }
            else if (data_all_hold_input == 's' && eTaskGetState(task_2_roller_handle) != eSuspended){
                printf("task #3 : roller should stop imidietly. cutter may continue\n");
                if(xQueueSend(roller_instruction_que,&roller_signal_stop,5)){
                    printf("task #3 : send  roller stop ins\n");
                }
                vTaskDelay(100);
                data_all_hold_input = NULL;
            }
        }

        vTaskDelay(5);
    }
}
void task_6_user_all_run_input(void *args){
    uint8_t data_all_run_input;
    uint8_t roller_auto = INSTR_ROLLER_AUTO;

    while (1) {

        if (gpio_get_level(ALL_RUN_INPUT) == 1){
            printf("task #6 : recieved command for all RUN input\n");
        }
        if (xQueueReceive(uart_instruction_que_all_run, &data_all_run_input, 5) == pdTRUE){
            vTaskDelay(2);
            printf("task #6 : recieved item is '%c'\n", data_all_run_input);
            if (data_all_run_input == 'r'){
                vTaskDelay(10);
                if (eTaskGetState(task_2_roller_handle) == eSuspended){
                    printf("task #6 : Resuming the roller and creating new instruction chain\n");
                    vQueueDelete(roller_instruction_que);
                    roller_instruction_que = xQueueCreate(10,sizeof(uint8_t));
                    vTaskDelay(10);

                    if (xQueueSend(roller_instruction_que, &data_all_run_input, 5) == pdTRUE)
                    {
                        vTaskDelay(20);
                        printf("task #6 : send item is '%c'\n",data_all_run_input);
                        vTaskResume(task_2_roller_handle);
                    }
                    else{
                    printf("task #6 : Failed to send message\n");
                    }
                }else if (eTaskGetState(task_2_roller_handle) != eSuspended){
                    printf("task #6 : roller is already running\n");
                
                }

                vTaskDelay(20);                
                data_all_run_input = NULL;
            }
        }

        vTaskDelay(5);
    }
}
void task_4_user_cut_input(void *args){
    uint8_t data_cut_input;
    while (1){
        if(gpio_get_level(USER_CUT_INPUT) == 1){
            printf("task #4 : recieved command for all hold input\n");
            vTaskResume(task_4_user_cut_handle);
        }
        if(xQueueReceive(uart_instruction_que_cutter_run,& data_cut_input,5) == pdTRUE){
            vTaskDelay(2);
            printf("task #4 : recieved item is '%c'\n",data_cut_input);
            if(data_cut_input == 'c'){
                vTaskResume(task_1_cutter);
                printf("task #4 :  task 1 cutter should run\n");
                data_cut_input = NULL;
            }
          }

        vTaskDelay(5);
        
    }
    

}
void task_5_user_roll_input(void *args){
    uint8_t data_roll_input;
    while (1){
        if(gpio_get_level(USER_CUT_INPUT) == 1){
            printf("task #5 : recieved command for all hold input\n");
        }
        if(xQueueReceive(uart_instruction_que_roller_run,&data_roll_input,5) == pdTRUE){
            vTaskDelay(2);
            printf("task #5 : recieved item '%c'\n",data_roll_input);
            if(data_roll_input == 'm'){
                vTaskResume(task_2_roller_handle);
                printf("task #5 :  task 2 roller should run\n");
                data_roll_input = NULL;
            }
        }
        vTaskDelay(5);
    }

}



void task_7_uart_listener(void *args){
    uint8_t data;
    while (1)
    {
        uart_read_bytes(UART_NUM_0, &data, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (data == 's')
        {
            printf("task #7 : recieved command to suspend alltask with char '%c'  function forwarded task_3\n",data);
            if(xQueueSend(uart_instruction_que_all_stop,&data,10) == pdTRUE){
                data = 0;
            }
            uart_flush(UART_NUM_0);
            vTaskDelay(5);
        }
        if(data == 'r'){
            printf("task #7 : recieved command to resume alltask with char '%c    function forwarded task_6'\n",data);
            if(xQueueSend(uart_instruction_que_all_run,&data,10) == pdTRUE){
                data = 0;
            }
            uart_flush(UART_NUM_0);
            vTaskDelay(5);
        }
        if (data == 'c'){
            printf("task #7 : recieved command to suspend alltask with char '%c'  function forwarded task_4\n",data);
            if(xQueueSend(uart_instruction_que_cutter_run,&data,10) == pdTRUE){
                data = 0;

            }
            uart_flush(UART_NUM_0);
            vTaskDelay(5);
        }
        if(data == 'm'){
            printf("task #7 : recieved command to resume alltask with char '%c    function forwarded task_5'\n",data);
            if(xQueueSend(uart_instruction_que_roller_run,&data,10) == pdTRUE){
                data = 0;
            }
            uart_flush(UART_NUM_0);
            vTaskDelay(5);
        }

    }
    vTaskDelete(NULL);
}
