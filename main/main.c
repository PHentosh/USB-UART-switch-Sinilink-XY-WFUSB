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

#define EX_UART_NUM UART_NUM_0
#define UARD_BAUD   115200
#define BUF_SIZE    UART_FIFO_LEN + 1
#define QUEUE_SIZE  UART_FIFO_LEN + 1

static QueueHandle_t uart0_queue;


static void uart_event_task(void *pvParameters)
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
                        ESP_LOGI(TAG, "ON");
                        gpio_set_level(GPIO_OUTPUT_IO_POWER, 1);
                    }
                    else if(data[0] == '0')
                    {
                        ESP_LOGI(TAG, "OFF");
                        gpio_set_level(GPIO_OUTPUT_IO_POWER, 0);
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
    gpio_config_t io_conf =
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

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
}