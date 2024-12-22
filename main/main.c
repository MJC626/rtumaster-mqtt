#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "simple_wifi_sta.h"
#include "agile_modbus.h"
#include "agile_modbus_rtu.h"
#include "uart_rtu.h"
#include "esp_http_server.h"
#include "cJSON.h"

static const char* TAG = "main";

// MQTT配置
#define MQTT_ADDRESS     "mqtt://52.172.156.83"     
#define MQTT_PORT        1883                        
#define MQTT_CLIENT      "mqttx_d11213"              
#define MQTT_USERNAME    "admin"                     
#define MQTT_PASSWORD    "Mjc200211"                  
#define MQTT_PUBLIC_TOPIC       "/test/topic1"       

// Modbus配置
#define MODBUS_MASTER_TASK_STACK_SIZE 4096
#define MAX_REGS 100  // 每种寄存器最大数量

// 事件组位定义
#define WIFI_CONNECT_BIT     BIT0
static EventGroupHandle_t s_wifi_ev = NULL;

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_is_mqtt_connected = false;

// Modbus主站缓冲区
static uint8_t master_send_buf[AGILE_MODBUS_MAX_ADU_LENGTH];
static uint8_t master_recv_buf[AGILE_MODBUS_MAX_ADU_LENGTH];

// Modbus配置参数结构体
typedef struct {
    uint8_t slave_addr;      // 从站地址
    uint16_t start_addr1;    // 第一组起始地址
    uint16_t reg_count1;     // 第一组寄存器数量
    uint16_t start_addr2;    // 第二组起始地址
    uint16_t reg_count2;     // 第二组寄存器数量
    uint32_t poll_interval;  // 轮询间隔(ms)
} modbus_config_t;

// 默认配置
static modbus_config_t modbus_config = {
    .slave_addr = 1,
    .start_addr1 = 0,
    .reg_count1 = 10,
    .start_addr2 = 10,
    .reg_count2 = 10,
    .poll_interval = 60
};

// 共享数据结构
typedef struct {
    uint16_t coils[MAX_REGS];
    uint16_t discrete_inputs[MAX_REGS];
    uint16_t holding_registers[MAX_REGS];
    uint16_t input_registers[MAX_REGS];
    bool input_ready;    
} modbus_data_t;

static modbus_data_t modbus_data = {0};

// Modbus采集任务
static void modbus_poll_task(void *pvParameters)
{
    agile_modbus_rtu_t ctx_rtu;
    agile_modbus_t *ctx = &ctx_rtu._ctx;
    agile_modbus_rtu_init(&ctx_rtu, master_send_buf, sizeof(master_send_buf), 
                         master_recv_buf, sizeof(master_recv_buf));

    while(1) {
        bool success = true;
        
        // 更新从站地址
        agile_modbus_set_slave(ctx, modbus_config.slave_addr);
        
        // 读取第一组输入寄存器
        int send_len = agile_modbus_serialize_read_input_registers(ctx, 
                        modbus_config.start_addr1, 
                        modbus_config.reg_count1);
        if (send_len > 0) {
            send_data(ctx->send_buf, send_len);
            int read_len = receive_data(ctx->read_buf, ctx->read_bufsz, 1000, 20);
            
            if (read_len > 0) {
                uint16_t reg_values[MAX_REGS];
                int rc = agile_modbus_deserialize_read_input_registers(ctx, read_len, reg_values);
                if (rc >= 0) {
                    memcpy(&modbus_data.input_registers[modbus_config.start_addr1], 
                           reg_values, 
                           sizeof(uint16_t) * modbus_config.reg_count1);
                    ESP_LOGI(TAG, "第一组输入寄存器数据已采集");
                } else {
                    ESP_LOGE(TAG, "第一组输入寄存器数据解析失败");
                    success = false;
                }
            } else {
                ESP_LOGE(TAG, "第一组输入寄存器读取超时");
                success = false;
            }
        } else {
            ESP_LOGE(TAG, "第一组输入寄存器请求打包失败");
            success = false;
        }

        // 读取第二组输入寄存器
        send_len = agile_modbus_serialize_read_input_registers(ctx, 
                    modbus_config.start_addr2, 
                    modbus_config.reg_count2);
        if (send_len > 0) {
            send_data(ctx->send_buf, send_len);
            int read_len = receive_data(ctx->read_buf, ctx->read_bufsz, 1000, 20);
            
            if (read_len > 0) {
                uint16_t reg_values[MAX_REGS];
                int rc = agile_modbus_deserialize_read_input_registers(ctx, read_len, reg_values);
                if (rc >= 0) {
                    memcpy(&modbus_data.input_registers[modbus_config.start_addr2], 
                           reg_values, 
                           sizeof(uint16_t) * modbus_config.reg_count2);
                    ESP_LOGI(TAG, "第二组输入寄存器数据已采集");
                } else {
                    ESP_LOGE(TAG, "第二组输入寄存器数据解析失败");
                    success = false;
                }
            } else {
                ESP_LOGE(TAG, "第二组输入寄存器读取超时");
                success = false;
            }
        } else {
            ESP_LOGE(TAG, "第二组输入寄存器请求打包失败");
            success = false;
        }

        modbus_data.input_ready = success;
        vTaskDelay(pdMS_TO_TICKS(modbus_config.poll_interval));
    }
}

// MQTT发布任务
static void mqtt_publish_task(void *pvParameters)
{
    char mqtt_pub_buff[512];
    
    while(1) {
        if(s_is_mqtt_connected && modbus_data.input_ready) {  
            snprintf(mqtt_pub_buff, sizeof(mqtt_pub_buff), 
                    "{\"type\":\"input\",\"data\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]}", 
                    modbus_data.input_registers[0],
                    modbus_data.input_registers[1],
                    modbus_data.input_registers[2],
                    modbus_data.input_registers[3],
                    modbus_data.input_registers[4],
                    modbus_data.input_registers[5],
                    modbus_data.input_registers[6],
                    modbus_data.input_registers[7],
                    modbus_data.input_registers[8],
                    modbus_data.input_registers[9],
                    modbus_data.input_registers[10],
                    modbus_data.input_registers[11],
                    modbus_data.input_registers[12],
                    modbus_data.input_registers[13],
                    modbus_data.input_registers[14],
                    modbus_data.input_registers[15],
                    modbus_data.input_registers[16],
                    modbus_data.input_registers[17],
                    modbus_data.input_registers[18],
                    modbus_data.input_registers[19]
                    );
            
            esp_mqtt_client_publish(s_mqtt_client, MQTT_PUBLIC_TOPIC,
                                 mqtt_pub_buff, strlen(mqtt_pub_buff), 1, 0);
            ESP_LOGI(TAG, "发布输入寄存器数据: %s", mqtt_pub_buff);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// 获取配置的处理函数
static esp_err_t get_config_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "slave_addr", modbus_config.slave_addr);
    cJSON_AddNumberToObject(root, "start_addr1", modbus_config.start_addr1);
    cJSON_AddNumberToObject(root, "reg_count1", modbus_config.reg_count1);
    cJSON_AddNumberToObject(root, "start_addr2", modbus_config.start_addr2);
    cJSON_AddNumberToObject(root, "reg_count2", modbus_config.reg_count2);
    cJSON_AddNumberToObject(root, "poll_interval", modbus_config.poll_interval);
    
    char *json_str = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

// 更新配置的处理函数
static esp_err_t update_config_handler(httpd_req_t *req)
{
    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        return ESP_FAIL;
    }
    content[ret] = '\0';

    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *slave_addr = cJSON_GetObjectItem(root, "slave_addr");
    cJSON *start_addr1 = cJSON_GetObjectItem(root, "start_addr1");
    cJSON *reg_count1 = cJSON_GetObjectItem(root, "reg_count1");
    cJSON *start_addr2 = cJSON_GetObjectItem(root, "start_addr2");
    cJSON *reg_count2 = cJSON_GetObjectItem(root, "reg_count2");
    cJSON *poll_interval = cJSON_GetObjectItem(root, "poll_interval");

    if (slave_addr) modbus_config.slave_addr = slave_addr->valueint;
    if (start_addr1) modbus_config.start_addr1 = start_addr1->valueint;
    if (reg_count1) modbus_config.reg_count1 = reg_count1->valueint;
    if (start_addr2) modbus_config.start_addr2 = start_addr2->valueint;
    if (reg_count2) modbus_config.reg_count2 = reg_count2->valueint;
    if (poll_interval) modbus_config.poll_interval = poll_interval->valueint;

    cJSON_Delete(root);
    
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// HTML页面处理函数
static esp_err_t get_html_handler(httpd_req_t *req)
{
    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[] asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_size);
    
    return ESP_OK;
}

// HTTP服务器URL配置
static const httpd_uri_t html = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = get_html_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t config_get = {
    .uri       = "/api/config",
    .method    = HTTP_GET,
    .handler   = get_config_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t config_post = {
    .uri       = "/api/config",
    .method    = HTTP_POST,
    .handler   = update_config_handler,
    .user_ctx  = NULL
};

// 启动HTTP服务器
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &html);
        httpd_register_uri_handler(server, &config_get);
        httpd_register_uri_handler(server, &config_post);
        ESP_LOGI(TAG, "HTTP服务器启动成功");
        return server;
    }

    ESP_LOGE(TAG, "HTTP服务器启动失败");
    return NULL;
}

static void aliot_mqtt_event_handler(void* event_handler_arg,
                                   esp_event_base_t event_base,
                                   int32_t event_id,
                                   void* event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    static TaskHandle_t modbus_task_handle = NULL;
    static TaskHandle_t mqtt_task_handle = NULL;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT已连接");
            s_is_mqtt_connected = true;
            // esp_mqtt_client_subscribe_single(s_mqtt_client, MQTT_SUBSCRIBE_TOPIC, 1);  //移除订阅
            
            // MQTT连接成功后创建任务
            if (modbus_task_handle == NULL) {
                xTaskCreate(modbus_poll_task,
                          "modbus_task",
                          MODBUS_MASTER_TASK_STACK_SIZE,
                          NULL,
                          5,
                          &modbus_task_handle);
            }
            if (mqtt_task_handle == NULL) {
                xTaskCreate(mqtt_publish_task,
                          "mqtt_task",
                          MODBUS_MASTER_TASK_STACK_SIZE,
                          NULL,
                          5,
                          &mqtt_task_handle);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT已断开");
            s_is_mqtt_connected = false;
            if (modbus_task_handle != NULL) {
                vTaskDelete(modbus_task_handle);
                modbus_task_handle = NULL;
            }
            if (mqtt_task_handle != NULL) {
                vTaskDelete(mqtt_task_handle);
                mqtt_task_handle = NULL;
            }
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT订阅成功, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT发布成功, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            printf("主题=%.*s\r\n", event->topic_len, event->topic);
            printf("数据=%.*s\r\n", event->data_len, event->data);
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT错误");
            break;
            
        default:
            break;
    }
}

void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {0};
    mqtt_cfg.broker.address.uri = MQTT_ADDRESS;
    mqtt_cfg.broker.address.port = MQTT_PORT;
    mqtt_cfg.credentials.client_id = MQTT_CLIENT;
    mqtt_cfg.credentials.username = MQTT_USERNAME;
    mqtt_cfg.credentials.authentication.password = MQTT_PASSWORD;
    
    ESP_LOGI(TAG,"MQTT配置->客户端ID:%s,用户名:%s,密码:%s",
             mqtt_cfg.credentials.client_id,
             mqtt_cfg.credentials.username,
             mqtt_cfg.credentials.authentication.password);
             
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, aliot_mqtt_event_handler, s_mqtt_client);
    esp_mqtt_client_start(s_mqtt_client);
}

void wifi_event_handler(WIFI_EV_e ev)
{
    if(ev == WIFI_CONNECTED)
    {
        xEventGroupSetBits(s_wifi_ev, WIFI_CONNECT_BIT);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    ESP_ERROR_CHECK(uart_init());
    
    s_wifi_ev = xEventGroupCreate();
    EventBits_t ev = 0;

    wifi_sta_init(wifi_event_handler);

    ev = xEventGroupWaitBits(s_wifi_ev, WIFI_CONNECT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    if(ev & WIFI_CONNECT_BIT)
    {
        mqtt_start();
        // 启动HTTP服务器
        start_webserver();
    }
    
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}