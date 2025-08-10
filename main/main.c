#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"

static const char *TAG = "USB-SW";

#define GPIO_OUTPUT_IO_GREEN    14
#define GPIO_OUTPUT_IO_BLUE     16
#define GPIO_OUTPUT_IO_POWER    5
#define GPIO_OUTPUT_PIN_SEL     ((1ULL<<GPIO_OUTPUT_IO_GREEN) | (1ULL<<GPIO_OUTPUT_IO_BLUE) | (1ULL<<GPIO_OUTPUT_IO_POWER))

#define GPIO_INPUT_IO_BUTTON    4
#define GPIO_INPUT_PIN_SEL      (1ULL<<GPIO_INPUT_IO_BUTTON)


#define EX_UART_NUM UART_NUM_0
#define UARD_BAUD   115200
#define BUF_SIZE    UART_FIFO_LEN + 1
#define QUEUE_SIZE  UART_FIFO_LEN + 1

static QueueHandle_t uart0_queue;
static QueueHandle_t power_queue;

typedef enum
{
    ON,
    OFF,
} state_t;

static void switch_task(void* _)
{
    state_t event;
    for (;;)
    {
        if (xQueueReceive(power_queue, (void*) &event, (portTickType) portMAX_DELAY))
        {
            switch (event) {
                case ON:
                    ESP_LOGI(TAG, "ON");
                    gpio_set_level(GPIO_OUTPUT_IO_POWER, 1);
                    break;
                case OFF:
                    ESP_LOGI(TAG, "OFF");
                    gpio_set_level(GPIO_OUTPUT_IO_POWER, 0);
                    break;
                default:
                    ESP_LOGE(TAG, "UNKNOWN event: %d", event);
                    break;
            }
        }
    }
}

static void gpio_isr_handler(void* _)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint8_t lvl = gpio_get_level(GPIO_OUTPUT_IO_POWER);
    state_t tmp = lvl ? OFF : ON;
    xQueueSendFromISR( power_queue, &tmp, &xHigherPriorityTaskWoken);
}

static void uart_event_task(void* _)
{
    uart_event_t event;
    uint8_t*     data = (uint8_t*) malloc(BUF_SIZE);

    for (;;)
    {
        if (xQueueReceive(uart0_queue, (void*) &event, (portTickType) portMAX_DELAY))
        {
            switch (event.type) {
                case UART_DATA:
                    bzero(data, BUF_SIZE);
                    uart_read_bytes(EX_UART_NUM, data, event.size, portMAX_DELAY);

                    if(data[0] == '1')
                    {
                        state_t tmp = ON;
                        xQueueSend(power_queue, &tmp, 0);
                    }
                    else if(data[0] == '0')
                    {
                        state_t tmp = OFF;
                        xQueueSend(power_queue, &tmp, 0);
                    }
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(data);
    data = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    power_queue = xQueueCreate(10, sizeof(state_t));

    gpio_config_t io_conf =
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_set_intr_type(GPIO_INPUT_IO_BUTTON, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INPUT_IO_BUTTON, gpio_isr_handler, (void*) NULL);

    uart_config_t uart_config =
    {
        .baud_rate = UARD_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_driver_install(EX_UART_NUM, BUF_SIZE, BUF_SIZE, QUEUE_SIZE, &uart0_queue, 0);
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    xTaskCreate(switch_task, "switch_task", 2048, NULL, 11, NULL);
}