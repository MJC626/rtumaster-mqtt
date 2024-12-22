// uart_rtu.h
#ifndef __uart_rtu_H
#define __uart_rtu_H

#include "driver/uart.h"

// DMA相关配置
#define UART_DMA_BUFFER_SIZE    256    // DMA缓冲区大小
#define UART_TX_DMA_CHANNEL    2      // 发送DMA通道
#define UART_RX_DMA_CHANNEL    3      // 接收DMA通道

int send_data(uint8_t *buf, int len);
int receive_data(uint8_t *buf, int bufsz, int timeout, int bytes_timeout);
int uart_init(void);

#endif

// uart_rtu.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "uart_rtu.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE UART_DMA_BUFFER_SIZE

static QueueHandle_t uart_queue;
static SemaphoreHandle_t rx_sem;

// 发送数据（使用DMA）
int send_data(uint8_t *buf, int len) {
    // 使用DMA方式发送数据
    return uart_write_bytes_with_break(UART_NUM, (const char*)buf, len, 100);
}

// 接收数据（使用DMA）
int receive_data(uint8_t *buf, int bufsz, int timeout, int bytes_timeout) {
    int len = 0;
    int rc;
    TickType_t start = xTaskGetTickCount();
    
    while (1) {
        if (xSemaphoreTake(rx_sem, pdMS_TO_TICKS(timeout)) == pdTRUE) {
            // 使用DMA方式读取数据
            rc = uart_read_bytes(UART_NUM, buf + len, bufsz, pdMS_TO_TICKS(bytes_timeout));
            if (rc > 0) {
                len += rc;
                bufsz -= rc;
                if (bufsz == 0)
                    break;
            } else if (rc == 0) {
                break;
            }
        } else {
            break;
        }
        
        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS >= timeout)
            break;
    }
    
    return len;
}

// UART事件处理任务
static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    size_t buffered_size;
    
    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    uart_get_buffered_data_len(UART_NUM, &buffered_size);
                    xSemaphoreGive(rx_sem);
                    break;
                    
                case UART_FIFO_OVF:
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                case UART_BUFFER_FULL:
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                default:
                    break;
            }
        }
    }
}

// UART初始化（启用DMA）
int uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // 配置UART参数
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    // 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 安装UART驱动，启用DMA
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, ESP_INTR_FLAG_IRAM));
    
    // 配置DMA
    uart_set_rx_timeout(UART_NUM, 3);  // 设置接收超时
    uart_set_rx_full_threshold(UART_NUM, 120);  // 设置RX FIFO满阈值
    
    // 创建信号量
    rx_sem = xSemaphoreCreateBinary();
    if (rx_sem == NULL) {
        ESP_LOGE("UART", "Failed to create rx semaphore");
        return -1;
    }
    
    // 创建UART事件处理任务
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    
    return ESP_OK;
}